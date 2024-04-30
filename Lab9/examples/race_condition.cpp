#include<iostream>
#include <thread>
#include <future>
#include <chrono>
#include <time.h>
#include <math.h>
#include <vector>


void increment_n_times(int* value, int n) {
    for(int i = 0; i < n; i++) {
        (*value)++;
    }
}

void decrement_n_times(int* value, int n) {
    for(int i = 0; i < n; i++) {
        (*value)--;
    }
}

void increment_and_decrement_n_times_race_condition(int*value, int n) {
    std::vector<std::future<void>> futures;
    auto inc_future = std::async(std::launch::async, increment_n_times, value, n);
    auto dec_future = std::async(std::launch::async, decrement_n_times, value, n);
    futures.push_back(std::move(inc_future));
    futures.push_back(std::move(dec_future));

    for(auto & future : futures) {
        future.wait();
    }
}

int main(int argc, char** argv) {
    int number = 0;
    int n = 10000;

    increment_n_times(&number, n);
    decrement_n_times(&number, n);
    std::cout<<"Value: "<<number<<std::endl;
    increment_and_decrement_n_times_race_condition(&number, n);
    std::cout<<"Value: "<<number<<std::endl;

    return 0;
}
