#include<iostream>
#include <thread>
#include <future>
#include <chrono>
#include <time.h>
#include <math.h>
#include <vector>
#include "timer.h"

/**
 *
 * @param n A number to check
 * @return The result of primality test
 */
bool is_prime(int n) {
    if (n<2) {
        return false;
    }

    if(n<4) {
        return true;
    }

    auto limit = sqrt(n);

    for(int i = 2; i <= limit; i++) {
        if(n % i == 0) {
            return false;
        }
    }

    return true;
}

/**
 * The function calculates the number of primes in given range
 * @param low Lower bound
 * @param high Upper bound
 * @return The number of primes in given range
 */

int primes_in_range(int low, int high) {
    int primes=0;
    for(int i = low; i<=high;i++) {
        if(is_prime(i)){
            primes++;
        }
    }

    return primes;
}
/**
 * Multithreaded function for calculating the number of primes in given range
 * @param low Lower bound of range
 * @param high Upper bound of range
 * @param threads Number of threads
 * @return Number of primes in given range
 */

int primes_in_range_multithreaded(int low, int high, int threads) {
    int primes = 0;
    double chunk = (high-low)/(double)threads;

    std::vector<std::future<int>> futures;

    for(int i=0; i<threads; i++) {
        auto future = std::async(std::launch::async, primes_in_range, (int)(i*chunk), (int)((i+1)*chunk)); //thread start upon creation
        futures.push_back(std::move(future));
    }

    for(auto & future : futures) {
        primes += future.get();
    }

    return primes;
}


int main(int argc, char** argv) {
    auto timer = Timer();
    int n = 10000000; //numbers to check for primality test
    int threads = 16; //number of threads

    timer.start();
    int single_primes = primes_in_range(0, n);
    timer.stop();

    int single_elapsed = timer.elapsed();

    timer.start();
    int multi_primes = primes_in_range_multithreaded(0, n, threads);
    timer.stop();

    int multi_elapsed = timer.elapsed();
    std::cout<<"Primes single-threaded: "<<single_primes<<std::endl;
    std::cout<<"Elapsed time single-threaded: "<<single_elapsed<<std::endl;
    std::cout<<"Primes multi-threaded: "<<multi_primes<<std::endl;
    std::cout<<"Elapsed time multi-threaded: "<<multi_elapsed<<std::endl;

    return 0;
}
