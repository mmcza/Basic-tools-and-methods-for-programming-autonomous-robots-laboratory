cmake_minimum_required(VERSION 3.14)
project(ParallenProgramming)

set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

set(WITH_OPENCV YES)
set(YES 1)
set(NO 0)

add_executable(future examples/future.cpp utils/timer.h)
add_executable(primes_loop examples/primes_loop.cpp utils/timer.h)
target_include_directories(primes_loop PUBLIC utils)


add_executable(thread examples/thread.cpp utils/timer.h)
add_executable(race_condition examples/race_condition.cpp utils/timer.h)
add_executable(safe_increment examples/safe_increment.cpp utils/timer.h)
add_executable(sum_in_range examples/sum_in_range.cpp utils/timer.h)


add_executable(task1 tasks/task1.cpp utils/timer.h)
target_include_directories(task1 PUBLIC utils)
target_link_libraries(task1)

add_executable(task2 tasks/task2.cpp utils/timer.h)
target_include_directories(task2 PUBLIC utils)
target_link_libraries(task2)



if(${WITH_OPENCV} EQUAL YES)
    find_package(OpenCV REQUIRED )

    add_executable(visual_race_condition examples/visual_race_condition.cpp utils/timer.h)
    target_include_directories(visual_race_condition PUBLIC utils)

    add_executable(visual_lock examples/visual_lock.cpp utils/timer.h)
    target_include_directories(visual_lock PUBLIC utils)

    add_executable(visual_multithreading examples/visual_multithreading.cpp utils/timer.h)
    target_include_directories(visual_multithreading PUBLIC utils)

    target_link_libraries( visual_race_condition ${OpenCV_LIBS} )
    target_link_libraries( visual_lock ${OpenCV_LIBS} )
    target_link_libraries( visual_multithreading ${OpenCV_LIBS} )

    add_executable(task3 tasks/task3.cpp utils/timer.h)
    target_include_directories(task3 PUBLIC utils)
    target_link_libraries(task3 ${OpenCV_LIBS} )
endif()
