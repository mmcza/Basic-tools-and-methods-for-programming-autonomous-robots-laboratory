#include<iostream>
#include <thread>
#include <future>
#include <chrono>
#include <time.h>
#include <math.h>
#include <vector>
#include "timer.h"
#include <opencv2/opencv.hpp>


void change_pixel(cv::Mat& image, char r, char g, char b, int x, int y) {
    auto& pixel = image.at<cv::Vec3b>(x, y);
    pixel[0] = b;
    pixel[1] = g;
    pixel[2] = r;
}

void colorize_pixel(cv::Mat& image, unsigned char r, unsigned char g, unsigned char b, int x, int y) {
    auto& pixel = image.at<cv::Vec3b>(x, y);

    pixel[0] = (unsigned char)(pixel[0]*0.8f + b *0.2f);
    pixel[1] = (unsigned char)(pixel[1]*0.8f + g *0.2f);
    pixel[2] = (unsigned char)(pixel[2]*0.8f + r *0.2f);
}

void change_rows(cv::Mat& image, unsigned char r, unsigned char g, unsigned char b, int start_row, int end_row) {
    int rows = image.rows;
    int cols = image.cols;
    int k=0;
    for(int i = start_row; i < end_row; i++) {
        for(int j = 0; j < cols; j++, k++) {
                change_pixel(image, r, g, b, i, j);
        }
            std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }
}

void colorize_single_thread(cv::Mat image) {
    int rows = image.rows;
    int cols = image.cols;
    int k=0;
    cv::namedWindow("Single-threaded colorizing");

    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < cols; j++, k++) {
            change_pixel(image, 255, 0, 0, i, j);
        }
        cv::imshow("Single-threaded colorizing", image);
        cv::waitKey(10);
    }

    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < cols; j++, k++) {
                change_pixel(image, 0, 255, 0, i, j);
        }
        cv::imshow("Single-threaded colorizing", image);
        cv::waitKey(10);
    }

    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < cols; j++, k++) {
            change_pixel(image, 0, 0, 255, i, j);
        }
        cv::imshow("Single-threaded colorizing", image);
        cv::waitKey(10);
    }

    cv::imshow("Single-threaded Final", image);
    cv::waitKey(0);

}

void colorize_multi_thread(cv::Mat image) {
    int rows = image.rows;
    int cols = image.cols;
    cv::namedWindow("Multi-threaded colorizing");
    std::vector<std::future<void>> futures;

    auto red_thread = std::async(std::launch::async, change_rows, std::ref(image), 255, 0, 0, 0, rows);

    auto blue_thread = std::async(std::launch::async, change_rows, std::ref(image), 0, 255, 0, 0, rows);

    auto green_thread = std::async(std::launch::async, change_rows, std::ref(image), 0, 0, 255, 0, rows);

    futures.push_back(std::move(red_thread));
    futures.push_back(std::move(blue_thread));
    futures.push_back(std::move(green_thread));

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

        cv::imshow("Multi-threaded colorizing", image);
        cv::waitKey(10);

    }


    cv::imshow("Multi-threaded Final", image);
    cv::waitKey(0);
}

int main(int argc, char** argv) {

    auto timer = Timer();
    int threads = 16;

    auto image = cv::imread("../assets/landscape.jpg");
    colorize_single_thread(image.clone());
    colorize_multi_thread(image.clone());

    return 0;
}
