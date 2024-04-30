#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <mutex>
#include <opencv2/opencv.hpp>

/**
 * Write a program which colorizes an image with 3 horizontal stripes:
 * - red in the upper part
 * - green in the middle part
 * - blue in the bottom part
 * Each stripe must be drawn for a given number of times and each of the color must be drawn simultaneously.
 * Every next drawn stripe make the color be more visible.
 *
 * Hint:
 * Make use of the visual_multithreading.cpp example!
 *
 * @return
 */

std::mutex mutex; // mutex for synchronization

int red_stripes = 2;
int green_stripes = 3;
int blue_stripes = 4;

void draw_stripes(cv::Mat& image, int start_row, int end_row, cv::Scalar color) {
    cv::Vec3b bgr_color(color[0], color[1], color[2]); // Extract BGR components and create Vec3b object
    for (int y = start_row; y < end_row; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            image.at<cv::Vec3b>(y, x) = bgr_color; // Assign Vec3b color
        }
    }
}

void colorize_multi_thread(cv::Mat image) {
    int height = image.rows;
    int stripe_height = height / (red_stripes + green_stripes + blue_stripes);

    std::vector<std::future<void>> futures;

    for (int i = 0; i < red_stripes; ++i) {
        int start_row = i * stripe_height;
        int end_row = (i + 1) * stripe_height;
        futures.emplace_back(std::async(std::launch::async, draw_stripes, std::ref(image), start_row, end_row, cv::Scalar(0, 0, 255)));
    }

    for (int i = 0; i < green_stripes; ++i) {
        int start_row = (red_stripes + i) * stripe_height;
        int end_row = (red_stripes + i + 1) * stripe_height;
        futures.emplace_back(std::async(std::launch::async, draw_stripes, std::ref(image), start_row, end_row, cv::Scalar(0, 255, 0)));
    }

    for (int i = 0; i < blue_stripes; ++i) {
        int start_row = (red_stripes + green_stripes + i) * stripe_height;
        int end_row = (red_stripes + green_stripes + i + 1) * stripe_height;
        futures.emplace_back(std::async(std::launch::async, draw_stripes, std::ref(image), start_row, end_row, cv::Scalar(255, 0, 0)));
    }

    // Wait for all tasks to complete
    for (auto& future : futures) {
        future.wait();
    }

    cv::imshow("Colorized Image", image);
    cv::waitKey(0);
}

int main() {
    auto image = cv::imread("../assets/landscape.jpg");
    if (image.empty()) {
        std::cerr << "Failed to load image!\n";
        return 1;
    }

    colorize_multi_thread(image.clone());

    return 0;
}
