
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