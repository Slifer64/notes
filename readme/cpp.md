
# Misc

- https://www.cppstories.com/2016/11/iife-for-complex-initialization/

    ```cpp
    void BuildStringTestIIFE2(std::string link, std::string text) 
    {
        const std::string html = [&] 
        {
            const auto& inText = text.empty() ? link : text;
            return "<a href=\"" + link + "\">" + inText + "</a>";
        }(); // call!

        std::cout << html << '\n';
    }
    ```

# Multithreading

- `std::thread`, `std::async`, `std::future`, `std::promise`, `std::packaged_task`

`std::promise` is where `std::future` come from. `std::future` is what allows you to retrieve a value that's been promised to you. When you call `get()` on a future, it waits until the owner of the `std::promise` with which it sets the value (by calling set_value on the promise). If the promise is destroyed before a value is set, and you then call `get()` on a future associated with that promise, you'll get a `std::broken_promise` exception because you were promised a value, but it's impossible for you to get one.

Suppose we have a function that takes some arguments:
```cpp
int foo(double, char, bool);
```
For our example function, we expect a `std::future<int>`.

- Option 1 (less control):
```cpp
// don't know if the function is exe­cu­ted concurrently, serially upon get(), or by some other black magic
auto fut = std::async(foo, 1.5, 'x', false);  // is a std::future<int>
// if (fut.wait_for(std::chrono::milliseconds(500)) == std::future_status::timeout) ...
int res = fut.get();
```

- Option 2 (more control):
```cpp
std::packaged_task<int(double, char, bool)> tsk(foo);
auto fut = tsk.get_future();    // is a std::future<int>
// The thread starts running immediately!
std::thread thr(std::move(tsk), 1.5, 'x', false);
int res = fut.get();
// detach or join the thread ...
```

- Option 3 (promise):
```cpp
auto res_promise = std::promise<int>();
auto my_task = [&res_promise](double d, char c, bool b)
{
    // Simple:
    // res_promise.set_value(foo(d, c, b));

    // Or to also account for exceptions
    try
    {
        auto res = foo(d, c, b);
        res_promise.set_value(res);
    }
    catch (std::exception &e)
    {
        res_promise.set_exception(std::make_exception_ptr(e));
    }
};
std::thread(my_task, 1.5, 'x', false).detach();
// if `set_exception()` was set, it will trigger here
int res = res_promise.get_future().get();
```

More generic implementation (custom implementation of `std::packaged_task`):
```cpp
template <typename> class my_task;

template <typename R, typename ...Args>
class my_task<R(Args...)>
{
    std::function<R(Args...)> fn;
    std::promise<R> pr;             // the promise of the result
public:
    template <typename ...Ts>
    explicit my_task(Ts &&... ts) : fn(std::forward<Ts>(ts)...) { }

    template <typename ...Ts>
    void operator()(Ts &&... ts)
    {
        pr.set_value(fn(std::forward<Ts>(ts)...));  // fulfill the promise
    }

    std::future<R> get_future() { return pr.get_future(); }
    // disable copy, default move
};

// Example usage:
my_task<int(double, std::string)> task(foo); // Create a task with foo function
std::thread([&]()
{
    task(1.5, 'x', false);
}).detach();
int res = task.get_future().get();
```

Notes on future:
- Only one future may be obtained!
- A `std::shared_future` object behaves like a `std::future` object, except that it can be copied, and that more than one shared_future can share ownership over their end of a shared state. They also allow the value in the shared state to be retrieved multiple times once ready.


For more details see https://stackoverflow.com/questions/11004273/what-is-stdpromise

# Variable number of arguments

- Variable arguments with different types
    ```cpp
    #include <iostream>
    #include <string>

    template <typename T>
    void func(T t) 
    {
        std::cout << t << std::endl ;
    }

    template<typename T, typename... Args>
    void func(T t, Args... args) // recursive variadic function
    {
        // std::cout << t <<std::endl;
        std::cout << __PRETTY_FUNCTION__ << ": " << t <<std::endl;

        func(args...) ;
    }


    int main()
    {
        func(1, "Hello", 2.5, 'a', std::string("World"));
    } 
    ```

- Variable arguments of the **same type**
    ```cpp
    #include <iostream>
    #include <string>
    #include <initializer_list>

    template <class T>
    void func2( std::initializer_list<T> list )
    {
        for( auto elem : list )
        {
            std::cout << elem << std::endl ;
        }
    }

    int main()
    {
        std::string
            str1( "Hello" ),
            str2( "world" );

        func2( {10, 20, 30, 40 }) ;
        func2( {str1, str2 } ) ;
    } 
    ```

# Parse command line args (parse cmd args)

```cpp
// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>

struct ConstrainedOptions
{
    double delta;
    double lambda;
    double tolerance;
    double time;
    unsigned int tries;
    double range;
};
void addConstrainedOptions(po::options_description &desc, struct ConstrainedOptions *options)
{
    desc.add_options()("delta,d", po::value<double>(&options->delta)->default_value(om::CONSTRAINED_STATE_SPACE_DELTA),
        "Step-size for discrete geodesic on manifold.");
    desc.add_options()("lambda", po::value<double>(&options->lambda)->default_value(om::CONSTRAINED_STATE_SPACE_LAMBDA),
        "Maximum `wandering` allowed during traversal. Must be greater than 1.");
    desc.add_options()("tolerance", po::value<double>(&options->tolerance)->default_value(om::CONSTRAINT_PROJECTION_TOLERANCE),
        "Constraint satisfaction tolerance.");
    desc.add_options()("time", po::value<double>(&options->time)->default_value(5.),
        "Planning time allowed.");
    desc.add_options()("tries", po::value<unsigned int>(&options->tries)->default_value(om::CONSTRAINT_PROJECTION_MAX_ITERATIONS),
        "Maximum number sample tries per sample.");
    desc.add_options()("range,r", po::value<double>(&options->range)->default_value(0),
        "Planner `range` value for planners that support this parameter. Automatically determined otherwise (when 0).");
}

bool output, bench;
enum SPACE_TYPE space = PJ;
std::vector<enum PLANNER_TYPE> planners = {RRT};
struct ConstrainedOptions c_opt;

void parseCmdArgs(int argc, char **argv)
{
    po::options_description desc("Options");
    desc.add_options()("help,h", 
        "Shows this help message.");
    desc.add_options()("output,o", po::bool_switch(&output)->default_value(false),
        "Dump found solution path (if one exists) in plain text and planning graph in GraphML to "
        "`sphere_path.txt` and `sphere_graph.graphml` respectively.");
    desc.add_options()("bench", po::bool_switch(&bench)->default_value(false),
        "Do benchmarking on provided planner list.");
    desc.add_options()("planner,p", po::value<std::vector<enum PLANNER_TYPE>>(planners)->multitoken(),
        "List of which motion planner to use (multiple if benchmarking, one if planning). Choose from:\n"
        "RRT (Default), RRT_I, RRTConnect, RRTConnect_I, RRTstar, EST, BiEST, ProjEST, BITstar");
    desc.add_options()("space,s", po::value<enum SPACE_TYPE>(space),
        "Choose which constraint handling methodology to use. One of:\n"
        "PJ - Projection (Default), "
        "AT - Atlas, "
        "TB - Tangent Bundle.");
    addConstrainedOptions(desc, &c_opt);

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        exit(0);
    }
}

bool argParse(int argc, char **argv, double *runTimePtr, optimalPlanner *plannerPtr, planningObjective *objectivePtr,
              std::string *outputFilePtr)
{
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()("help,h", "produce help message")(
        "runtime,t", bpo::value<double>()->default_value(1.0),
        "(Optional) Specify the runtime in seconds. Defaults to 1 and "
        "must be greater than 0.")("planner,p", bpo::value<std::string>()->default_value("RRTstar"),
                                   "(Optional) Specify the optimal planner to use, defaults to RRTstar if not given. "
                                   "Valid options are AITstar, "
                                   "BFMTstar, BITstar, CForest, EITstar, EIRMstar, FMTstar, InformedRRTstar, PRMstar, RRTstar, "
                                   "and SORRTstar.")  // Alphabetical order
        ("objective,o", bpo::value<std::string>()->default_value("PathLength"),
         "(Optional) Specify the optimization objective, defaults to PathLength if not given. Valid options are "
         "PathClearance, PathLength, ThresholdPathLength, and WeightedLengthAndClearanceCombo.")  // Alphabetical order
        ("file,f", bpo::value<std::string>()->default_value(""),
         "(Optional) Specify an output path for the found solution path.")(
            "info,i", bpo::value<unsigned int>()->default_value(0u),
            "(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG. Defaults to WARN.");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    // Check if the help flag has been given:
    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        return false;
    }

    // Set the log-level
    unsigned int logLevel = vm["info"].as<unsigned int>();

    // Switch to setting the log level:
    if (logLevel == 0u)
    {
        ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
    }
    else if (logLevel == 1u)
    {
        ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
    }
    else if (logLevel == 2u)
    {
        ompl::msg::setLogLevel(ompl::msg::LOG_DEBUG);
    }
    else
    {
        std::cout << "Invalid log-level integer." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the runtime as a double
    *runTimePtr = vm["runtime"].as<double>();

    // Sanity check
    if (*runTimePtr <= 0.0)
    {
        std::cout << "Invalid runtime." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the specified planner as a string
    std::string plannerStr = vm["planner"].as<std::string>();

    // Map the string to the enum
    if (boost::iequals("AITstar", plannerStr))
    {
        *plannerPtr = PLANNER_AITSTAR;
    }
    else if (boost::iequals("BFMTstar", plannerStr))
    {
        *plannerPtr = PLANNER_BFMTSTAR;
    }
    else
    {
        std::cout << "Invalid planner string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the specified objective as a string
    std::string objectiveStr = vm["objective"].as<std::string>();

    // Map the string to the enum
    if (boost::iequals("PathClearance", objectiveStr))
    {
        *objectivePtr = OBJECTIVE_PATHCLEARANCE;
    }
    else if (boost::iequals("PathLength", objectiveStr))
    {
        *objectivePtr = OBJECTIVE_PATHLENGTH;
    }
    else
    {
        std::cout << "Invalid objective string." << std::endl << std::endl << desc << std::endl;
        return false;
    }

    // Get the output file string and store it in the return pointer
    *outputFilePtr = vm["file"].as<std::string>();

    // Looks like we parsed the arguments successfully
    return true;
}
```

```cmake
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)

add_executable(<exec_name> <src>.cpp)
target_link_libraries(<exec_name> PRIVATE Boost::program_options)
```

# Print format
`Cheatsheet`: https://hackingcpp.com/cpp/libs/fmt.html

Example:
```cpp
#include <iostream>
#include <vector>
#include <string>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <armadillo>
#include <Eigen/Dense>

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    std::vector<std::string> names = {
        "Andreas",
        "Antonis",
        "Pavlos",
        "Georege"
    };

    std::cerr << fmt::format("{}", fmt::join(names, ", ")) << "\n";

    std::vector<double> v {1.2, 5.6, 7.8};
    arma::vec av = {5, 8, 2, -2, 7};
    Eigen::VectorXd ev(4);
    ev << 8, -7, -6, 1;

    fmt::print("{}\n", v);
    fmt::print("[{}]\n", fmt::join(av, "|"));
    fmt::print("{}\n", ev);

    return 0;
}
```

For `CMakeLists.txt`:
```cmake
find_package(fmt REQUIRED)
add_executable(<exec_name> src/<source>.cpp)
target_link_libraries(<exec_name> PRIVATE fmt::fmt <other libs>)
```

# Dictionary with generic values

Similarly can be done using `<boost/variant.hpp>`.
```cpp
#include <vector>
#include <string>
#include <string>
#include <unordered_map>
#include <variant>

using RtsiTypeVariant = std::variant<bool, uint8_t, uint16_t, uint32_t, uint64_t, int32_t, double, vector3d_t, vector6d_t, vector6int32_t, vector6uint32_t>;

std::unordered_map<std::string, RtsiTypeVariant> value_table_;
std::vector<std::string> recipe_list_;

for (auto item : recipe_list_)
{
    RtsiTypeVariant &value = value_table_[item];
    // bool, uint8_t, uint16_t, uint32_t, uint64_t, int32_t, double, vector3d_t, vector6d_t, vector6int32_t, vector6uint32_t
    if (std::holds_alternative<bool>(value))
    {
        value = (bool)package[offset];
        offset++;
    }
    else if (std::holds_alternative<uint8_t>(value))
    {
        value = (uint8_t)package[offset];
        offset++;
    }
    // ...
    else if (std::holds_alternative<vector3d_t>(value))
    {
        UTILS::EndianUtils::unpack<double, 3>(package, offset, std::get<vector3d_t>(value));
    }
    // ...
    else if (std::holds_alternative<vector6uint32_t>(value))
    {
        UTILS::EndianUtils::unpack<uint32_t, 6>(package, offset, std::get<vector6uint32_t>(value));
    }
    else
    {
        return false;
    }
}
```