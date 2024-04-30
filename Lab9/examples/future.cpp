// future example
#include <iostream>       // std::cout
#include <future>         // std::thread

/**
 * Function calculating sum of the next integers in the given range.
 * @param begin Lower bound of the given range.
 * @param end Upper bound of the given range.
 * @return Sum of next integers.
 */
int sum_in_range(int begin, int end)
{
    int sum = 0;
    for(int i=begin; i<=end; i++) {
        sum+=i;
    }
    std::cout<<"Function sum in range has finished.\n";
    return sum;
}

int main()
{
    int sum;
    int begin = 0;
    int end=100;
    std::future<int> f = std::async(sum_in_range, begin, end); //We create an asynchronous routine (thread) and pass the arguments.

    std::cout << "f is now executing concurrently...\n";

    sum=f.get();                // This is a blocking method. Pauses until thread finishes. Once it finishes the result is returned.

    std::cout << "The arithmetic sum of the next numbers from "<< begin<<" to "<< end<<" is equal to "<<sum<< ".\n";

    return 0;
}