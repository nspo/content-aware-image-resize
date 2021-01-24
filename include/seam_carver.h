#ifndef CONTENT_AWARE_IMAGE_RESIZE_SEAM_CARVER_H
#define CONTENT_AWARE_IMAGE_RESIZE_SEAM_CARVER_H

#include <opencv2/opencv.hpp>

class SeamCarver {
public:
    enum class EnergyCalculationType {
        RGB_Gradient1,
        Gray_Gradient1
    };

    explicit SeamCarver(const cv::Mat& _image, const EnergyCalculationType _energyCalcType = EnergyCalculationType::RGB_Gradient1)
                        :
                        image(_image.clone()),
                        energyCalcType(_energyCalcType) {
    }

    [[nodiscard]]
    int width() const {
        return image.cols;
    }

    [[nodiscard]]
    int height() const {
        return image.rows;
    }

    // determine energy of pixel using gradient of RGB values
    double energy(const int row, const int col) {
        if (row < 0 || row >= height() || col < 0 || col >= width()) {
            throw std::invalid_argument("Invalid coordinates");
        }

        if (row == 0 || row == height() - 1 || col == 0 || col == width() - 1) {
            // border pixels have static value
            return maxEnergyVal();
        }

        if (energyCache.size() != static_cast<size_t>(width())*height()) {
            energyCache.resize(width()*height());
            // fill with NaN
            std::fill(energyCache.begin(), energyCache.end(), std::numeric_limits<double>::quiet_NaN());
        }

        if (!std::isnan(energyCache[coordinatesToId(row, col)])) {
            // value is cached
            return energyCache[coordinatesToId(row, col)];
        }

        const cv::Vec3b& rgbRowP1 = image.at<cv::Vec3b>(row + 1, col);
        const cv::Vec3b& rgbRowM1 = image.at<cv::Vec3b>(row - 1, col);

        const cv::Vec3b& rgbColP1 = image.at<cv::Vec3b>(row, col + 1);
        const cv::Vec3b& rgbColM1 = image.at<cv::Vec3b>(row, col - 1);

        double total_energy = 0;

        if (energyCalcType == EnergyCalculationType::RGB_Gradient1) {
            const int rRow = rgbRowP1[2] - rgbRowM1[2];
            const int gRow = rgbRowP1[1] - rgbRowM1[1];
            const int bRow = rgbRowP1[0] - rgbRowM1[0];
            const double deltaRow2 = rRow * rRow + gRow * gRow + bRow * bRow;

            const int rCol = rgbColP1[2] - rgbColM1[2];
            const int gCol = rgbColP1[1] - rgbColM1[1];
            const int bCol = rgbColP1[0] - rgbColM1[0];
            const double deltaCol2 = rCol * rCol + gCol * gCol + bCol * bCol;

            total_energy = std::sqrt(deltaRow2 + deltaCol2);
        } else {
            // Gray_Gradient1
            const double grayRowP1 = 0.299 * rgbRowP1[2] + 0.587 * rgbRowP1[1] + 0.114 * rgbRowP1[0];
            const double grayRowM1 = 0.299 * rgbRowM1[2] + 0.587 * rgbRowM1[1] + 0.114 * rgbRowM1[0];
            const double deltaRow2 = (grayRowP1 - grayRowM1)*(grayRowP1 - grayRowM1);

            const double grayColP1 = 0.299 * rgbColP1[2] + 0.587 * rgbColP1[1] + 0.114 * rgbColP1[0];
            const double grayColM1 = 0.299 * rgbColM1[2] + 0.587 * rgbColM1[1] + 0.114 * rgbColM1[0];
            const double deltaCol2 = (grayColP1 - grayColM1)*(grayColP1 - grayColM1);

            total_energy = std::sqrt(deltaRow2 + deltaCol2);
        }

        // cache end result
        energyCache[coordinatesToId(row, col)] = total_energy;
        return energyCache[coordinatesToId(row, col)];
    }

    // determine sequence of row indices for horizontal seam
    std::vector<int> findHorizontalSeam() {
        // setup shortest path search without explicit EdgeWeightedDigraph etc.
        std::vector<double> distTo(width() * height() + 1, std::numeric_limits<double>::infinity());
        std::vector<int> edgeTo(width() * height() + 1, -1);

        // virtual vertex (connected to all pixels in the first column)
        const int vLeft = width() * height();
        distTo[vLeft] = 0;

        // connect vLeft to first col
        for (int row = 0; row < height(); ++row) {
            const int colLeft = 0;
            const int v = coordinatesToId(row, colLeft);
            relax(distTo, edgeTo, vLeft, v, energy(row, colLeft));
        }

        // relax pixels with edge to pixels in the next col to the right
        // (leverages the topological order of the implicit digraph)
        for (int col = 0; col + 1 < width(); ++col) { // skip last col
            const int colRight = col + 1;
            for (int row = 0; row < height(); ++row) {
                for (int rowRight = std::max(0, row - 1); rowRight <= std::min(height() - 1, row + 1);
                     ++rowRight) {
                    relax(distTo, edgeTo, coordinatesToId(row, col), coordinatesToId(rowRight, colRight),
                          energy(rowRight, colRight));
                }
            }
        }

        // identify pixel in last col with minimal dist to vLeft
        int vLastCol = -1;
        double distToLastCol = std::numeric_limits<double>::infinity();
        for (int row = 0; row < height(); ++row) {
            const int col = width() - 1;
            if (distToLastCol > distTo[coordinatesToId(row, col)]) {
                distToLastCol = distTo[coordinatesToId(row, col)];
                vLastCol = coordinatesToId(row, col);
            }
        }

        // save result in array of row coordinates
        std::vector<int> rowCoordinates(width());
        for (int v = vLastCol; v != vLeft; v = edgeTo[v]) {
            int col = idToCol(v);
            rowCoordinates[col] = idToRow(v);
        }

        return rowCoordinates;
    }

    // determine sequence of col indices for vertical seam
    std::vector<int> findVerticalSeam() {
        // setup shortest path search without explicit EdgeWeightedDigraph etc.
        std::vector<double> distTo(width() * height() + 1, std::numeric_limits<double>::infinity());
        std::vector<int> edgeTo(width() * height() + 1, -1);

        // virtual vertex (connected to all pixels in the first row)
        const int vTop = width() * height();
        distTo[vTop] = 0;

        // connect vTop to first row
        for (int col = 0; col < width(); ++col) {
            const int rowTop = 0;
            const int v = coordinatesToId(rowTop, col);
            relax(distTo, edgeTo, vTop, v, energy(rowTop, col));
        }

        // relax pixels with edge to pixels in the next row below
        // (leverages the topological order of the implicit digraph)
        for (int row = 0; row + 1 < height(); ++row) { // skip last row
            const int rowBelow = row + 1;
            for (int col = 0; col < width(); ++col) {
                for (int colBelow = std::max(0, col - 1); colBelow <= std::min(width() - 1, col + 1);
                     ++colBelow) {
                    relax(distTo, edgeTo, coordinatesToId(row, col), coordinatesToId(rowBelow, colBelow),
                          energy(rowBelow, colBelow));
                }
            }
        }

        // identify pixel in last row with minimal dist to vTop
        int vLastRow = -1;
        double distToLastRow = std::numeric_limits<double>::infinity();
        for (int col = 0; col < width(); ++col) {
            const int row = height() - 1;
            if (distToLastRow > distTo[coordinatesToId(row, col)]) {
                distToLastRow = distTo[coordinatesToId(row, col)];
                vLastRow = coordinatesToId(row, col);
            }
        }

        // save result in array of col coordinates
        std::vector<int> colCoordinates(height());
        for (int v = vLastRow; v != vTop; v = edgeTo[v]) {
            int row = idToRow(v);
            colCoordinates[row] = idToCol(v);
        }

        return colCoordinates;
    }

    // return copy of image where both seams are marked red
    cv::Mat imageWithMarkedSeams(const std::vector<int>& horizSeam,
                                 const std::vector<int>& vertSeam) {
        if (!validHorizontalSeam(horizSeam)) {
            throw std::invalid_argument("Invalid horizontal seam");
        }
        if (!validVerticalSeam(vertSeam)) {
            throw std::invalid_argument("Invalid vertical seam");
        }

        cv::Mat copiedImg = image.clone();

        for (int col=0; col < copiedImg.cols; ++col) {
            const int row = horizSeam[col];
            auto& pixel = copiedImg.at<cv::Vec3b>(row, col);
            pixel[0] = 0;
            pixel[1] = 0;
            pixel[2] = 255;
        }

        for (int row=0; row < copiedImg.rows; ++row) {
            const int col = vertSeam[row];
            auto& pixel = copiedImg.at<cv::Vec3b>(row, col);
            pixel[0] = 0;
            pixel[1] = 0;
            pixel[2] = 255;
        }

        return copiedImg;
    }

    // return energy image where brightness values correspond to calculated energy of the pixel in original image
    [[nodiscard]]
    cv::Mat energyImage() {
        cv::Mat newImage(image.rows, image.cols, CV_64FC1);

        for (int col=0; col<image.cols; ++col) {
            for (int row=0; row<image.rows; ++row) {
                const double energyVal = energy(row, col)/maxEnergyVal();
                newImage.at<double>(row, col) = energyVal;
            }
        }
        return newImage;
    }

    void removeHorizontalSeam(const std::vector<int>& horizontalSeam) {
        if (!validHorizontalSeam(horizontalSeam)) {
            throw std::invalid_argument("Invalid horizontal seam");
        }

        for (int col = 0; col < width(); ++col) {
            for (int row = horizontalSeam[col]; row+1 < height(); ++row) {
                image.at<cv::Vec3b>(row, col) = image.at<cv::Vec3b>(row+1, col);
            }
        }

        image = image.rowRange(0, image.rows - 1).colRange(0, image.cols);
    }

    void removeHorizontalSeam() {
        removeHorizontalSeam(findHorizontalSeam());
    }

    void removeVerticalSeam(const std::vector<int>& verticalSeam) {
        if (verticalSeam.size() != static_cast<size_t>(height())) {
            throw std::invalid_argument("Invalid seam size");
        }

        for (int row = 0; row < height(); ++row) {
            for (int col = verticalSeam[row]; col+1 < width(); ++col) {
                // shift image
                image.at<cv::Vec3b>(row, col) = image.at<cv::Vec3b>(row, col+1);
            }
        }

        image = image.rowRange(0, image.rows).colRange(0, image.cols-1);
    }

    void removeVerticalSeam() {
        removeVerticalSeam(findVerticalSeam());
    }

    [[nodiscard]]
    cv::Mat getImage() const {
        // TODO: worry about modifications
        return image;
    }

private:
    cv::Mat image;
    const EnergyCalculationType energyCalcType;
    std::vector<double> energyCache; // cache for energy values

    [[nodiscard]]
    double maxEnergyVal() const {
        if (energyCalcType == EnergyCalculationType::RGB_Gradient1) {
            return 624.6198844097104;
        } else if (energyCalcType == EnergyCalculationType::Gray_Gradient1) {
            return 360.62445840513925;//std::sqrt(std::pow(255.0, 2) + std::pow(255.0, 2));
        } else {
            return 1000;
        }
    }

    [[nodiscard]]
    int coordinatesToId(int row, int col) const {
        return row * width() + col;
    }

    [[nodiscard]]
    int idToRow(int id) const {
        return id / width();
    }

    [[nodiscard]]
    int idToCol(int id) const {
        return id % width();
    }

    // check whether argument is a valid horizontal seam
    [[nodiscard]]
    bool validHorizontalSeam(const std::vector<int>& horizSeam) const {
        if (horizSeam.size() != static_cast<size_t>(width())) {
            return false;
        }

        for (size_t i=0; i<horizSeam.size(); ++i) {
            if (i>0) {
                // not first element
                if (std::abs(horizSeam[i] - horizSeam[i-1]) > 1) {
                    return false;
                }
            }
            if (horizSeam[i] < 0 || horizSeam[i] >= height()) {
                return false;
            }
        }
        return true;
    }

    // check whether argument is a valid vertical seam
    [[nodiscard]]
    bool validVerticalSeam(const std::vector<int>& vertSeam) const {
        if (vertSeam.size() != static_cast<size_t>(height())) {
            return false;
        }

        for (size_t i=0; i < vertSeam.size(); ++i) {
            if (i>0) {
                // not first element
                if (std::abs(vertSeam[i] - vertSeam[i - 1]) > 1) {
                    return false;
                }
            }
            if (vertSeam[i] < 0 || vertSeam[i] >= width()) {
                return false;
            }
        }
        return true;
    }

    // helper function to relax edge without explicit digraph
    static void relax(std::vector<double>& distTo, std::vector<int>& edgeTo, const int vFrom, const int vTo, const double weight) {
        if (distTo[vTo] > distTo[vFrom] + weight) {
            edgeTo[vTo] = vFrom;
            distTo[vTo] = distTo[vFrom] + weight;
        }
    }
};

#endif //CONTENT_AWARE_IMAGE_RESIZE_SEAM_CARVER_H
