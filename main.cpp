#include <iostream>
#include <opencv2/opencv.hpp>
#include "seam_carver.h"

void sample_run(const cv::Mat& img);

void print_usage(char* argv[]) {
    std::cerr<<"Usage: "<<argv[0]<<" [-s] IMAGE_FILENAME\n";
}

int main(int argc, char* argv[]) {
    // Parse arguments
    if (argc < 2 || argc > 3) {
        print_usage(argv);
        return 1;
    }

    bool only_sample_run = false;
    const char* image_filename;
    if (argc == 3) {
        if (strcmp(argv[1], "-s") == 0) {
            only_sample_run = true;
            image_filename = argv[2];
        } else {
            print_usage(argv);
            return 1;
        }
    } else {
        // argc == 2
        image_filename = argv[1];
    }

    cv::Mat img = cv::imread(image_filename, cv::IMREAD_COLOR);
    if (img.empty()) {
        std::cerr<<"Error opening image file '"<<image_filename<<"'\n";
        return 1;
    }

    // Sample run
    if (only_sample_run) {
        sample_run(img);
        return 0;
    }

    // Actually open interactive window
    cv::namedWindow(image_filename, cv::WINDOW_NORMAL);
    cv::resizeWindow(image_filename, 800, 800);

    SeamCarver sc(img);

    bool markSeamRed = true;
    while (sc.getImage().rows > 1 && sc.getImage().cols > 1) {
        const auto horizontalSeam = sc.findHorizontalSeam();
        const auto verticalSeam = sc.findVerticalSeam();

        cv::imshow(image_filename,
                   markSeamRed ? sc.pictureWithMarkedSeams(horizontalSeam, verticalSeam) : sc.getImage());
        const int key = cv::waitKey(0);
        if (key == 82 || key == 84) {
            // up / down
            sc.removeHorizontalSeam(horizontalSeam);
        } else if (key == 81 || key == 83) {
            // left / right
            sc.removeVerticalSeam(verticalSeam);
        } else if (key == 114) {
            // r
            // toggle red seam
            markSeamRed = !markSeamRed;
        } else if (key == 27 || key == -1) {
            // escape / close
            break;
        }

    }

    return 0;
}

void sample_run(const cv::Mat& img) {
    SeamCarver sc(img);
    auto start = std::chrono::system_clock::now();
    for (int i=0; i<10; i++) {
        sc.removeHorizontalSeam();
        sc.removeVerticalSeam();
    }
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout<<"Sample run took "<<elapsed.count()<<" ms\n";
}