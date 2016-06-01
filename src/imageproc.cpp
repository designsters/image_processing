#include "imageproc.h"
#include <set>

std::vector<std::vector<cv::Point>> ImageProc::FindPerimeter(const cv::Mat& region) {

    CV_Assert(region.depth() == CV_8U);

    std::vector<std::vector<cv::Point> > perimeters;
    std::set<int> found_perimeters; // set of all points of found perimeters

    // Finds first point of the region.
    auto find_region = [&region](const cv::MatConstIterator_<uchar>& begin) -> cv::MatConstIterator_<uchar> {
        for (auto it = begin; it != region.end<uchar>(); it++) {
            cv::Point current = it.pos();
            cv::Point prev(current.x - 1, current.y);
            cv::Point next(current.x + 1, current.y);
            // If previous or next point is marked and current one is marked
            if (PointIsMarked(region, current) && !(PointIsMarked(region, prev) && PointIsMarked(region, next))) {
                return it;
            }
        }
        return region.end<uchar>();
    };

    for (auto it = find_region(region.begin<uchar>()); it != region.end<uchar>(); it = find_region(it + 1)) {
        cv::Point pos = it.pos();

        if (found_perimeters.find(pos.x * region.cols + pos.y) == found_perimeters.end()) { // If found region is new.
            std::vector<cv::Point> contour = FindPerimeter(region, pos);
            for (const cv::Point& p : contour) {
                found_perimeters.insert(p.x * region.cols + p.y);
            }
            perimeters.push_back(contour);
        }
    }
    return perimeters;
}

std::vector<cv::Point> ImageProc::FindPerimeter(const cv::Mat& region, const cv::Point& start_pos) {

    CV_Assert(region.depth() == CV_8U);

    std::vector<cv::Point> perimeter;
    cv::Point current_pos = start_pos;
    cv::Point last_pos(start_pos.x - 1, start_pos.y);
    do {
        perimeter.push_back(current_pos);

        auto rotate_left = [](const cv::Point& p) { return cv::Point(p.y, -p.x); };
        auto rotate_right = [](const cv::Point& p) { return cv::Point(-p.y, p.x); };

        int turns_number = 1;
        // Follow the border of the region, left turn is prioritized.
        cv::Point new_pos = rotate_left(current_pos - last_pos);
        for (; turns_number < 5 && !PointIsMarked(region, current_pos + new_pos); turns_number++) {
            new_pos = rotate_right(new_pos);
        }

        if (turns_number > 4) { // If there are more than 4 turns it's one point region
            break;
        }

        last_pos = current_pos;
        current_pos += new_pos;
    } while (current_pos != start_pos);

    return perimeter;
}

std::vector<cv::Point> ImageProc::SmoothPerimeter(const std::vector<cv::Point>& perimeter, int smooth_factor) {

	return std::vector<cv::Point>();
}

bool ImageProc::PointBelongsToImage(const cv::Mat& image, const cv::Point& p) {
    return p.y >= 0 && p.y < image.rows && p.x >= 0 && p.x < image.cols;
};

bool ImageProc::PointIsMarked(const cv::Mat& image, const cv::Point& p) {
    return PointBelongsToImage(image, p) && image.at<uchar>(p) != 0;
};
