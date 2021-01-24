#include "seam_carver.h"

template<typename T>
bool vectorsEqual(const std::vector<T>& v1, const std::vector<T>& v2, int64_t idxStart = -1, int64_t idxPastLast = -1) {
    if (v1.size() != v2.size()) {
        std::cerr << "Two given vectors have different sizes\n";
        return false;
    }

    if (idxStart < 0) idxStart = 0;
    if (idxPastLast < 0) idxPastLast = v1.size();
    for (int64_t i=idxStart; i < idxPastLast; ++i) {
        if (v1.at(i) != v2.at(i)) {
            std::cerr << "Two given vectors are not equal\n";
            std::cerr << "Index: " << i << ", v1: " << v1.at(i) << ", v2: " << v2.at(i) << "\n";
            return false;
        }
    }

    return true;
}

int main(int argc, char* argv[]) {
    (void) argc;
    (void) argv;
    cv::Mat img = cv::imread("6x5.png", cv::IMREAD_COLOR);
    if (img.empty()) {
        std::cerr << "Could not open test file! Execute this in main directory.\n";
        return 1;
    }

    SeamCarver sc(img);
    const std::vector<int> refHorizontalSeam = {-1, 2, 1, 2, 1, -1};
    const auto horizontalSeam = sc.findHorizontalSeam();
    if (vectorsEqual(horizontalSeam, refHorizontalSeam, 1, refHorizontalSeam.size()-1)) {
        std::cout << "Horizontal seam matches\n";
    } else {
        std::cerr << "Horizontal seam does not match\n";
        return 1;
    }

    const std::vector<int> refVerticalSeam = {-1, 4, 3, 2, -1};
    const auto verticalSeam = sc.findVerticalSeam();
    if (vectorsEqual(verticalSeam, refVerticalSeam, 1, refVerticalSeam.size()-1)) {
        std::cout << "Vertical seam matches\n";
    } else {
        std::cerr << "Vertical seam does not match\n";
        return 1;
    }

    return 0;
}