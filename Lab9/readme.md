## Information

> [!NOTE]
> Based on [Rafał Staszak's repository](https://github.com/RafalStaszak/ParallelProgramming)

A repostiory of basic multi-threading concepts in C++:
* threads and futures
* mutexes and locks
* thread synchronization
* race condition

The **examples** folder contains sample implementations of aforementioned concepts. Each of the **.cpp** file
may be run individually. 

## Prerequisites

* Linux (pthread)
* C++11
* Visual examples require OpenCV installation

In order to run only **non-visual** examples without OPENCV installation change the following
setting in the **CMakeLists.txt** file:

```cmake
set(WITH_OPENCV YES) # type YES or NO
```

The project can be loaded with whatever IDE supporting **cmake**.

