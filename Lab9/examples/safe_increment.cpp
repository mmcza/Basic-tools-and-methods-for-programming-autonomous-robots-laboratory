#include <thread>
#include <mutex>
#include <iostream>

int value = 0;
std::mutex mutex_increment;  // protects safe_increment

void safe_increment()
{
    const std::lock_guard<std::mutex> lock(mutex_increment); // lock_guard locks execution of the function until it finishes.
    value++;
}

int main()
{
    std::thread t1(safe_increment);
    std::thread t2(safe_increment);

    t1.join(); // Wait for the threads to end
    t2.join();

    std::cout << "Value: " << value << '\n'; // Value: 2
}