#include <iostream>
// #include <timer.h>
#include <thread>
#include <future>
#include <mutex>
#include <vector>
#include <chrono>
#include <string>
#include <chrono>

/**
 * Calculate the sum of of 0's in the range from 1 to n. For example the number 10230 has 2 zeros.
 *
 * Hints:
 * How single-threaded program should work:
 * It loops through each number from 1 to n
 *
 *
 * How multi-threaded program should work:
 * Split the range from 1 to n int equal chunks
 * Each thread calculates the sum of zeros from Begin to End
 *
 * Write your own functions, mutexes, locks!
 */

int n = 1000000;

int run_single_threaded() {
    int number_of_zeros = 0;
    for(int i=1; i<=n; i++)
    {
        std::string num_as_string = std::to_string(i);
        for(int j=0; j < num_as_string.size(); j++)
        {
            if (num_as_string[j]=='0')
            {
                number_of_zeros += 1;
            }
        }
    }
    return number_of_zeros;
}

int count_zeros_in_range(int start, int end) {
    int count = 0;
    for (int i = start; i <= end; ++i) {
        std::string num_as_string = std::to_string(i);
        for (size_t j = 0; j < num_as_string.size(); ++j) {
            if (num_as_string[j] == '0') {
                count++;
            }
        }
    }
    return count;
}

int run_multi_threaded(int threads) {
    int number_of_zeros = 0;
    std::vector<std::thread> all_threads;
    int segment_size = n / threads;
    int start = 1;
    for (int i = 0; i < threads; ++i) {
        int end = (i == threads - 1) ? n : start + segment_size - 1;
        all_threads.emplace_back([start, end, &number_of_zeros] {
            int zeros_in_segment = count_zeros_in_range(start, end);
            number_of_zeros += zeros_in_segment;
        });
        start = end + 1;
    }
    for (auto& thread : all_threads) {
        thread.join();
    }
    return number_of_zeros;
}

int zeros_in_number(int number) {
    /*
     * implement that
     */
    return -1;
}

int main() {
    auto start = std::chrono::high_resolution_clock::now();
    int threads = 8;
    int zeros;

    zeros = run_single_threaded();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "Number of zeros in the range from 1 to : " << n << " equals " << zeros << ".\n";
    std::cout << "Execution time: " << elapsed.count() << " milliseconds.\n";

    start = std::chrono::high_resolution_clock::now();
    zeros = run_multi_threaded(threads);
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "Number of zeros in the range from 1 to : " << n << " equals " << zeros << ".\n";
    std::cout << "Execution time: " << elapsed.count() << " milliseconds.\n";

    std::cout << "Complete the task\n";
    return 0;
}