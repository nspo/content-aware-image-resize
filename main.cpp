#include <iostream>
#include <opencv2/opencv.hpp>
#include "seam_carver.h"
#include "argagg.h"

void sample_run(const cv::Mat& img);

int main(int argc, char* argv[]) {
    // Parse arguments
    argagg::parser argparser {{
                           {"help", {"-h", "--help"}, "Show this help message", 0},
                                     {"energy_type", {"-t", "--energy-type"}, "Energy calculation type (0 or 1)", 1},
                                     {"energy_image", {"-e", "--energy-image"}, "Only show energy image", 0},
                                     {"sample_run", {"-s", "--sample-run"}, "Only do sample run", 0}
                             }};
    argagg::parser_results args;
    try {
        args = argparser.parse(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }

    if (args["help"] || args.pos.size() != 1 ||
        (args["energy_type"] && (args["energy_type"].as<int>() < 0 || args["energy_type"].as<int>() > 1))) {
        std::cerr << "Usage: "<<argv[0]<<" [options] IMAGE_FILENAME\n" << argparser;
        return EXIT_SUCCESS;
    }

    const SeamCarver::EnergyCalculationType energyCalcType =
            !args["energy_type"] || args["energy_type"].as<int>() == 0
            ? SeamCarver::EnergyCalculationType::RGB_Gradient1
            : SeamCarver::EnergyCalculationType::Gray_Gradient1;

    const char* image_filename = args.as<const char*>(0);

    cv::Mat img = cv::imread(image_filename, cv::IMREAD_COLOR);
    if (img.empty()) {
        std::cerr<<"Error opening image file '"<<image_filename<<"'\n";
        return 1;
    }
    std::cout<<"Opened image file "<<image_filename<<"\n";

    // Sample run
    if (args["sample_run"]) {
        sample_run(img);
        return 0;
    }

    // Actually open interactive window
    cv::namedWindow(image_filename, cv::WINDOW_NORMAL);
    cv::resizeWindow(image_filename, 800, 800);

    SeamCarver sc(img, energyCalcType);

    if (args["energy_image"]) {
        cv::imshow(image_filename, sc.energyImage());
        cv::waitKey(0);
        return 0;
    }

    bool markSeamRed = true;
    while (sc.getImage().rows > 1 && sc.getImage().cols > 1) {
        const auto horizontalSeam = sc.findHorizontalSeam();
        const auto verticalSeam = sc.findVerticalSeam();

        cv::imshow(image_filename,
                   markSeamRed ? sc.imageWithMarkedSeams(horizontalSeam, verticalSeam) : sc.getImage());
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