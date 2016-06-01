#ifndef _IMAGEPROC_H_
#define _IMAGEPROC_H_

#include <opencv2/core/core.hpp>
#include <vector>
#include <queue>

namespace ImageProc {

    /*
    Finds contiguous region of the nearby pixels that are similar in color.
    params: image -
            target - point search starts working from
            upper_bound - maximal difference between colours of the target pixel and other pixels in the region
            diff - maximal difference between colours of two adjacent pixels in the region
    returns: matrix in which non zero elements belong to the region
    */
    template <typename T, int cn>
    cv::Mat FindRegion(const cv::Mat& image, const cv::Point& target, const cv::Vec<T, cn>& upper_bound, const cv::Vec<T, cn>& diff);

    /*
    Finds all perimetres of a region.
    params: region -
    returns: vector of perimetres, where each perimeter is a vector of points
    */
    std::vector<std::vector<cv::Point> > FindPerimeter(const cv::Mat& region);

    /*
    Finds perimeter which starts from start_pos point.
    params: region -
            start_pos - initial point of a perimeter
    returns: vector of points perimeter consists of
    */
    std::vector<cv::Point> FindPerimeter(const cv::Mat& region, const cv::Point& start_pos);

	/*
	Smooths perimeter
	*/
	std::vector<cv::Point> SmoothPerimeter(const  std::vector<cv::Point>& perimeter, int smooth_factor);


    /*
    Draws pixels on the image with given colour.
    */
    template <typename T, int cn>
    void DumpPixels(cv::Mat& image, const cv::Mat& pixels, const cv::Vec<T, cn>& colour);

    /*
    Draws pixels on the image with given colour.
    */
    template <typename T, int cn>
    void DumpPixels(cv::Mat& image, const std::vector<std::vector<cv::Point> >& pixels, const cv::Vec<T, cn>& colour);

    /*
    Checks whether the point is inside the image.
    */
    bool PointBelongsToImage(const cv::Mat& image, const cv::Point& p);

    /*
    Checks whether the point does not equal to zero.
    */
    bool PointIsMarked(const cv::Mat& image, const cv::Point& p);
}



template <typename T, int cn>
cv::Mat ImageProc::FindRegion(const cv::Mat& image, const cv::Point& target, const cv::Vec<T, cn>& upper_bound, const cv::Vec<T, cn>& diff) {
    typedef cv::Vec<T, cn> Vec;

    cv::Mat region = cv::Mat::zeros(image.size(), CV_8U);
    Vec target_colour = image.at<Vec>(target); // Initial colour to be flood filled.

    std::queue<cv::Point> queue;
    region.at<uchar>(target) = 255; // Mark the target point.
    queue.push(target);

    while (!queue.empty()) {
        cv::Point p = queue.front();
        queue.pop();

        Vec current_colour = image.at<Vec>(p); // Colour of the current point.

        // Checks whether the colour is within the diff range and upper bound range.
        auto colour_is_replaceable = [&target_colour, &current_colour, &upper_bound, &diff](const Vec& colour) -> bool {
            for (int i = 0; i < colour.rows; i++)
                if (abs(colour[i] - current_colour[i]) > diff[i] || abs(colour[i] - target_colour[i]) > upper_bound[i])
                    return false;
            return true;
        };

        // Checks whether the point is unmarked and has target colour and if so marks it.
        auto mark = [&image, &region, &queue, &colour_is_replaceable](const cv::Point& point) {
            if (region.at<uchar>(point) == 0 && colour_is_replaceable(image.at<Vec>(point))) {
                region.at<uchar>(point) = 255; // Mark point.
                queue.push(point);
            }
        };

        if (p.y > 0)
            mark(cv::Point(p.x, p.y - 1));
        if (p.y < image.rows - 1)
            mark(cv::Point(p.x, p.y + 1));
        if (p.x > 0)
            mark(cv::Point(p.x - 1, p.y));
        if (p.x < image.cols - 1)
            mark(cv::Point(p.x + 1, p.y));
    }

    return region;
}

template <typename T, int cn>
void ImageProc::DumpPixels(cv::Mat& image, const cv::Mat& pixels, const cv::Vec<T, cn>& colour) {

    CV_Assert(image.size() == pixels.size());
    CV_Assert(pixels.depth() == CV_8U);

    for (int i = 0; i < image.rows * image.cols; i++) {
        if (pixels.at<uchar>(i) != 0) {
            image.at<cv::Vec<T, cn>>(i) = colour;
        }
    }
}

template <typename T, int cn>
void ImageProc::DumpPixels(cv::Mat& image, const std::vector<std::vector<cv::Point> >& pixels, const cv::Vec<T, cn>& colour) {
    for (const std::vector<cv::Point>& v : pixels) {
        for (const cv::Point& p : v) {
            image.at<cv::Vec<T, cn>>(p) = colour;
        }
    }
}

#endif //_IMAGEPROC_H_
