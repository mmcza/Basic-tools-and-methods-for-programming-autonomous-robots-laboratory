// thread example
#include <iostream>       // std::cout
#include <thread>         // std::thread

/**
* Function calculating sum of the next integers in the given range.
* @param result A variable reference in which the result will be stored.
* @param begin Lower bound of the given range.
* @param end Upper bound of the given range.
*/
void sum_in_range(int& result, int begin, int end)
{
    int sum = 0;
    for(int i=begin; i<=end; i++) {
        sum+=i;
    }
    result = sum;
    std::cout<<"Function sum in range has finished.\n";
}

int main()
{
    int sum;
    int begin = 0;
    int end=100;
    std::thread t (sum_in_range, std::ref(sum), begin, end); //We need to pass a reference to the function.

    std::cout << "t is now executing concurrently...\n";

    t.join();                // This is a blocking method. Pauses until thread finishes.

    std::cout << "The arithmetic sum of the next numbers from "<< begin<<" to "<< end<<" is equal to "<<sum<< ".\n";

    return 0;
}