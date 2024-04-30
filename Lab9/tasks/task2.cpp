#include <iostream>
// #include <timer.h>
#include <thread>
#include <future>
#include <mutex>
#include <vector>
#include <chrono>

/**
 * The goal of program is to carry out all the tasks. The execution time of a single taskis given in the "time_per_task"
 * list. Write two programs, both single and multi threaded ones, which can complete all the tasks.
 *
 * Hints:
 * How single-threaded program should work:
 * It loops through the list "time_per_task".
 * On each iteration the function "task" is executed.
 *
 * How multi-threaded program should work:
 * Each thread seeks for uncompleted tasks.
 * After finding an uncompleted task the thread blocks the access to that task. (It cannot be done twice by any
 * other thred).
 * After all the tasks have been completed all the threads stop their execution.
 *
 * Write your own functions, mutexes, locks!
 */

std::vector<int> time_per_task{100, 400, 300, 800, 600, 300, 900, 700, 100, 400, 800, 300, 200, 500, 900};
std::vector<bool> completed_tasks(time_per_task.size(), false);
std::vector<bool> pending_tasks(time_per_task.size(), false); //may be useful if a task is started but not completed yet


void task(int time, int task) {
    std::this_thread::sleep_for (std::chrono::milliseconds(time));
    completed_tasks[task] = true;
}

void run_single_threaded() {
    for(int i=0; i<time_per_task.size();i++)
    {
        task(time_per_task[i], i);
    }
}

void run_multi_threaded(int threads) {
    std::vector<std::thread> thread_pool;
    int tasks_per_thread = time_per_task.size() / threads;
    int remaining_tasks = time_per_task.size() % threads;
    int task_index = 0;

    for (int i = 0; i < threads; i++) {
        int tasks_to_assign = tasks_per_thread + (i < remaining_tasks ? 1 : 0);
        thread_pool.emplace_back([&task_index, tasks_to_assign]() {
            for (int j = 0; j < tasks_to_assign; j++) {
                task(time_per_task[task_index], task_index);
                task_index++;
            }
        });
    }

    for (auto& thread : thread_pool) {
        thread.join();
    }
}

void reset_tasks() {
    for(auto&& task : completed_tasks) {
        task = false;
    }
}

int main() {
    auto start = std::chrono::high_resolution_clock::now();
    int threads = 8;

    run_single_threaded();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "Task has been completed for one thread! Execution time: " << elapsed.count() << " milliseconds.\n";

    reset_tasks();

    start = std::chrono::high_resolution_clock::now();
    run_multi_threaded(threads);
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "Task has been completed for " << threads << " threads! Execution time: " << elapsed.count() << " milliseconds.\n";

    std::cout << "Complete the task\n";
    return 0;
}