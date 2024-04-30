#ifndef TIMER_H
#define TIMER_H

#include <chrono>

/**
 * A timer class for measuring execution time.
 */
class Timer
{
private:
     std::chrono::steady_clock::time_point start_time;
     std::chrono::steady_clock::time_point end_time;

     bool started = false;

public:
    /**
     *
     * @return Elapsed time in milliseconds.
     */
     int elapsed() {
         if(started) {
             end_time =  std::chrono::steady_clock::now();
             return std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
         } else {
             return std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time).count();
         }
     }
    /**
     * Stops the timer.
     */
     void stop() {
         if(started) {
             end_time =  std::chrono::steady_clock::now();
             started=false;
         }
     }
    /**
     * Starts the timer.
     */
     void start() {
         if(!started){
             start_time =  std::chrono::steady_clock::now();
             started = true;
         }
     }
};

#endif // TIMER_H
