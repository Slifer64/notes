```cpp
// Get param `ros_name` from `node`, and if it fails, try to read it from an std::msg::String topic with the name `ros_name`

#include <moveit/rdf_loader/synchronized_string_parameter.h>

SynchronizedStringParameter urdf_ssp_;

std::shared_ptr<rclcpp::Node> node;
std::string ros_name = "robot_description";
bool default_continuous_value = false;
double default_timeout = 10.0;

urdf_string_ = urdf_ssp_.loadInitialValue(
      node, ros_name, [this](const std::string& new_urdf_string) { return urdfUpdateCallback(new_urdf_string); },
      default_continuous_value, default_timeout);
```

```cpp
// Move ownership of dynamically allocated memory
std::unique_ptr<urdf::Model> urdf = std::make_unique<urdf::Model>();
if (!urdf->initString(urdf_string_)) return false;
urdf_ = std::move(urdf);
```

```cpp
//  Load the file to a string using an efficient memory allocation technique
std::ifstream stream(path.c_str());
stream.seekg(0, std::ios::end);
buffer.reserve(stream.tellg());
stream.seekg(0, std::ios::beg);
buffer.assign((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
stream.close();
```

```cpp
// Compile xacro
std::string buffer;
const std::string path;
std::vector<std::string> xacro_args;

std::string cmd = "ros2 run xacro xacro ";
for (const std::string& xacro_arg : xacro_args) cmd += xacro_arg + " ";
cmd += path;

#ifdef _WIN32
    FILE* pipe = _popen(cmd.c_str(), "r");
#else
    FILE* pipe = popen(cmd.c_str(), "r");
#endif

if (!pipe)
{
    RCLCPP_ERROR(getLogger(), "Unable to load path");
    return false;
}

char pipe_buffer[128];
while (!feof(pipe))
{
    if (fgets(pipe_buffer, 128, pipe) != nullptr) buffer += pipe_buffer;
}
```

```cpp
// concatenate paths
std::string package_path = ament_index_cpp::get_package_share_directory(package_name);

std::filesystem::path path(package_path);
path = path / relative_path;
```

```cpp
// pointer definitions macro
#define RCLCPP_SHARED_PTR_DEFINITIONS(...) \
  __RCLCPP_SHARED_PTR_ALIAS(__VA_ARGS__) \
  __RCLCPP_MAKE_SHARED_DEFINITION(__VA_ARGS__)

#define __RCLCPP_SHARED_PTR_ALIAS(__VA_ARGS__...) using SharedPtr = std::shared_ptr<__VA_ARGS__>; using ConstSharedPtr = std::shared_ptr<const __VA_ARGS__>;

#define __RCLCPP_MAKE_SHARED_DEFINITION(__VA_ARGS__...) template<typename ... Args> static std::shared_ptr<__VA_ARGS__> make_shared(Args && ... args) { return std::make_shared<__VA_ARGS__>(std::forward<Args>(args) ...); }

// Example usage:
RCLCPP_SHARED_PTR_DEFINITIONS(ABBSystemHardware)
// Result:
using SharedPtr = std::shared_ptr<ABBSystemHardware>; 
using ConstSharedPtr = std::shared_ptr<const ABBSystemHardware>;
template<typename ... Args> static std::shared_ptr<ABBSystemHardware> make_shared(Args && ... args) { return std::make_shared<ABBSystemHardware>(std::forward<Args>(args) ...); }
```

```cpp
// string concat in DEFINE
#define STRING_JOIN(arg1, arg2) arg1 ## arg2

STRING_JOIN(foo, bar) // --> foobar
```

```cpp
// Output vector of strings separated by delimiter
std::vector<std::string> vec = {"apple", "banana", "orange", "grape"};
std::ostringstream out;
std::copy(vec.begin(), vec.end(), std::ostream_iterator<std::string>(out, " - "));
```

```cpp
// loop through dictionary elements
std::map<std::string, double> data = {
    {"foo", 4.226},
    {"bar", 4.226},
    {"bazz", 4.226},
}

// special iterator member functions for objects
for (auto it = data.begin(); it != data.end(); ++it)
    std::cout << it.key() << " : " << it.value() << "\n";

// the same code as range for
for (auto& el : data)
    std::cout << el.key() << " : " << el.value() << "\n";

// even easier with structured bindings (C++17)
for (auto& [key, value] : data)
    std::cout << key << " : " << value << "\n";
```

```c++
// RPY to quaternion
tf2::Quaternion zero_orientation;
zero_orientation.setRPY(0, 0, 0);
const geometry_msgs::msg::Quaternion zero_orientation_msg = tf2::toMsg(zero_orientation);
```

```c++
// remove default class member from class using `delete`
private:
  // non-copyable
  RealtimePublisher(const RealtimePublisher &) = delete;
  RealtimePublisher & operator=(const RealtimePublisher &) = delete;
```

```c++
// using with template argument
template <class Msg>
using RealtimePublisherSharedPtr = std::shared_ptr<RealtimePublisher<Msg>>;
```

```cpp
// parse xml
#include <tinyxml2.h>

tinyxml2::XMLDocument doc;
if (!doc.Parse(urdf.c_str()) && doc.Error()) throw throw std::runtime_error("invalid URDF passed in to robot parser");

// Find robot tag
const tinyxml2::XMLElement * robot_it = doc.RootElement();
if (std::string("robot") != robot_it->Name()) throw std::runtime_error("the robot tag is not root element in URDF");

// see https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/src/component_parser.cpp#L602
```

```cpp
// Get ros params from another node
auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(my_node, "remote_node_name");
while (!parameters_client->wait_for_service(1s)) {
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
    rclcpp::shutdown();
  }
  RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
}
auto parameters = parameters_client->get_parameters({"remote_param1", "remote_param2"});
```

```cpp
template<class... Args>
void print(Args... args)
{
    (std::cout << ... << args) << "\n";
}
print(1, ':', " Hello", ',', 2.37, " ", "World!");
```

```cpp
// Check if template arguments is of specific type, e.g. std::vector<T>
template<typename T>
struct is_std_vector : std::false_type {};

template<typename T, typename Alloc>
struct is_std_vector<std::vector<T, Alloc>> : std::true_type {};

template<typename T>
void my_function()
{
    if constexpr (is_std_vector<T>::value)
    {
        std::cout << "T is std::vector" << std::endl;
    }
    else
    {
        std::cout << "T is not std::vector" << std::endl;
    }
}

int main()
{
    my_function<int>();  // Output: T is not std::vector
    my_function<std::vector<double>>();  // Output: T is std::vector
    my_function<std::vector<int>>();  // Output: T is std::vector

    return 0;
}
```