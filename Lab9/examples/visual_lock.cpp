#include<iostream>
#include <thread>
#include <future>
#include <chrono>
#include <time.h>
#include <math.h>
#include <vector>
#include "timer.h"
#include <opencv2/opencv.hpp>

std::mutex mutex;

void change_pixel(cv::Mat& image, char r, char g, char b, int x, int y) {
    auto& pixel = image.at<cv::Vec3b>(x, y);
    pixel[0] = b;
    pixel[1] = g;
    pixel[2] = r;
}

void colorize_pixel(cv::Mat& image, unsigned char r, unsigned char g, unsigned char b, int x, int y) {
    auto& pixel = image.at<cv::Vec3b>(x, y);

    pixel[0] = (unsigned char)(pixel[0]*0.5f + b *0.5f);
    pixel[1] = (unsigned char)(pixel[1]*0.5f + g *0.5f);
    pixel[2] = (unsigned char)(pixel[2]*0.5f + r *0.5f);
}

void change_rows(cv::Mat& image, unsigned char r, unsigned char g, unsigned char b, int start_row, int end_row) {
    int rows = image.rows;
    int cols = image.cols;
    int k=0;
    for(int i = start_row; i < end_row; i++) {
        for(int j = 0; j < cols; j++, k++) {
                colorize_pixel(image, r, g, b, i, j);
        }
            std::this_thread::sleep_for (std::chrono::milliseconds(10));

    }
}

void change_rows_lock(cv::Mat& image, unsigned char r, unsigned char g, unsigned char b, int start_row, int end_row) {
    std::lock_guard<std::mutex> guard(mutex);
    int rows = image.rows;
    int cols = image.cols;
    int k=0;
    for(int i = start_row; i < end_row; i++) {
        for(int j = 0; j < cols; j++, k++) {
                colorize_pixel(image, r, g, b, i, j);
        }
            std::this_thread::sleep_for (std::chrono::milliseconds(10));

    }
}

void colorize_single_thread(cv::Mat image) {
    int rows = image.rows;
    int cols = image.cols;
    int k=0;
    cv::namedWindow("Single-threaded colorizing");

    for(int i = 0; i < image.rows; i++) {
        for(int j = 0; j < cols; j++, k++) {
            if(i<image.rows/3) {
                colorize_pixel(image, 255, 0, 0, i, j);
            } else if(i<2*image.rows/3) {
                colorize_pixel(image, 0, 255, 0, i, j);
            } else {
                colorize_pixel(image, 0, 0, 255, i, j);
            }

        }
        if(k % 50 == 0) {
            cv::imshow("Single-threaded colorizing", image);
            cv::waitKey(50);
        }

    }

    cv::imshow("Single-threaded Final", image);
    cv::waitKey(0);

}

void colorize_multi_thread_lock(cv::Mat image) {
    int rows = image.rows;
    int cols = image.cols;
    cv::namedWindow("Multi-threaded colorizing with lock");
    std::vector<std::future<void>> futures;

    auto red_thread_1 = std::async(std::launch::async, change_rows_lock, std::ref(image), 255, 0, 0, 0, rows/3);

    auto blue_thread_1 = std::async(std::launch::async, change_rows_lock, std::ref(image), 0, 255, 0, rows/3, 2*rows/3);

    auto green_thread_1 = std::async(std::launch::async, change_rows_lock, std::ref(image), 0, 0, 255, 2*rows/3, rows);

    auto red_thread_2 = std::async(std::launch::async, change_rows_lock, std::ref(image), 255, 0, 0, rows/3, 2*rows/3);

    auto blue_thread_2 = std::async(std::launch::async, change_rows_lock, std::ref(image), 0, 255, 0, 2*rows/3, rows);

    auto green_thread_2 = std::async(std::launch::async, change_rows_lock, std::ref(image), 0, 0, 255, 0, rows/3);

    auto red_thread_3 = std::async(std::launch::async, change_rows_lock, std::ref(image), 255, 0, 0, 2*rows/3, rows);

    auto blue_thread_3 = std::async(std::launch::async, change_rows_lock, std::ref(image), 0, 255, 0, 0, rows/3);

    auto green_thread_3 = std::async(std::launch::async, change_rows_lock, std::ref(image), 0, 0, 255, rows/3, 2*rows/3);

    futures.push_back(std::move(red_thread_1));
    futures.push_back(std::move(blue_thread_1));
    futures.push_back(std::move(green_thread_1));
    futures.push_back(std::move(red_thread_2));
    futures.push_back(std::move(blue_thread_2));
    futures.push_back(std::move(green_thread_2));
    futures.push_back(std::move(red_thread_3));
    futures.push_back(std::move(blue_thread_3));
    futures.push_back(std::move(green_thread_3));

    while(true) {
        bool pending_threads = 0;
        for(auto f = futures.begin(); f!=futures.end();f++) {
            if(f->wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
                pending_threads++;
            }
        }

        if(pending_threads == 0) {
            break;
        }

        cv::imshow("Multi-threaded colorizing with lock", image);
        cv::waitKey(10);

    }


    cv::imshow("Multi-threaded Final with lock", image);
    cv::waitKey(0);
}

void colorize_multi_thread_no_lock(cv::Mat image) {
    int rows = image.rows;
    int cols = image.cols;
    cv::namedWindow("Multi-threaded colorizing no lock");
    std::vector<std::future<void>> futures;

    auto red_thread_1 = std::async(std::launch::async, change_rows, std::ref(image), 255, 0, 0, 0, rows/3);

    auto blue_thread_1 = std::async(std::launch::async, change_rows, std::ref(image), 0, 255, 0, rows/3, 2*rows/3);

    auto green_thread_1 = std::async(std::launch::async, change_rows, std::ref(image), 0, 0, 255, 2*rows/3, rows);

    auto red_thread_2 = std::async(std::launch::async, change_rows, std::ref(image), 255, 0, 0, rows/3, 2*rows/3);

    auto blue_thread_2 = std::async(std::launch::async, change_rows, std::ref(image), 0, 255, 0, 2*rows/3, rows);

    auto green_thread_2 = std::async(std::launch::async, change_rows, std::ref(image), 0, 0, 255, 0, rows/3);

    auto red_thread_3 = std::async(std::launch::async, change_rows, std::ref(image), 255, 0, 0, 2*rows/3, rows);

    auto blue_thread_3 = std::async(std::launch::async, change_rows, std::ref(image), 0, 255, 0, 0, rows/3);

    auto green_thread_3 = std::async(std::launch::async, change_rows, std::ref(image), 0, 0, 255, rows/3, 2*rows/3);

    futures.push_back(std::move(red_thread_1));
    futures.push_back(std::move(blue_thread_1));
    futures.push_back(std::move(green_thread_1));
    futures.push_back(std::move(red_thread_2));
    futures.push_back(std::move(blue_thread_2));
    futures.push_back(std::move(green_thread_2));
    futures.push_back(std::move(red_thread_3));
    futures.push_back(std::move(blue_thread_3));
    futures.push_back(std::move(green_thread_3));

    while(true) {
        bool pending_threads = 0;
        for(auto f = futures.begin(); f!=futures.end();f++) {
            if(f->wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
                pending_threads++;
            }
        }

        if(pending_threads == 0) {
            break;
        }

        cv::imshow("Multi-threaded colorizing no lock", image);
        cv::waitKey(10);

    }


    cv::imshow("Multi-threaded Final no lock", image);
    cv::waitKey(0);
}

int main(int argc, char** argv) {

    auto timer = Timer();

    auto image = cv::imread("../assets/landscape.jpg");
    colorize_multi_thread_no_lock(image.clone());
    colorize_multi_thread_lock(image.clone());

    return 0;
}
