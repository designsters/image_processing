#include "imageproc.h"
#include <set>
#include <numeric>

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

std::vector<cv::Point> FillGaps(const std::vector<cv::Point>& perimeter) {
    std::vector<cv::Point> new_vec;

    cv::Point last_entry = perimeter[perimeter.size() - 1];

    for (const cv::Point& i : perimeter) {

        // if gap is bigger than 1 pixel fill it
        for (cv::Point diff = i - last_entry; abs(diff.x) > 1 || abs(diff.y) > 1; diff = i - last_entry) {
            cv::Point p = last_entry;

            if (abs(diff.x) > 1) {
                if (diff.x > 0)
                    p.x += 1;
                if (diff.x < 0)
                    p.x -= 1;
                p.y = i.y;
            }
            if (abs(diff.y) > 1) {
                if (diff.y > 0)
                    p.y += 1;
                if (diff.y < 0)
                    p.y -= 1;
                p.x = i.x;
            }

            new_vec.push_back(p);
            last_entry = p;
        }

        new_vec.push_back(i);
    }

    return new_vec;
}

std::vector<cv::Point> RemoveSuccessiveDuplicates(const std::vector<cv::Point>& perimeter) {
    std::vector<cv::Point> new_vec;

    cv::Point last_entry = perimeter[perimeter.size() - 1];
    for (const cv::Point& i : perimeter) {
        if (i != last_entry) {
            new_vec.push_back(i);
            last_entry = i;
        }
    }

    return new_vec;
}

double Gaussian(int x, double m, double s)
{
    return (1. / (s * sqrt(2 * 3.141592))) * exp(-0.5 * pow((x - m) / s, 2.0));
};

std::vector<cv::Point> ImageProc::SmoothPerimeter(const std::vector<cv::Point>& perimeter, double smooth_factor) {

    std::vector<cv::Point> smoothed_perimeter(perimeter.size());

    size_t core_size = 2 * (smooth_factor * 3 * sqrt(2 * 3.141592) / 4.);
    if (core_size % 2 == 0)
        core_size += 1;

    std::vector<double> core(core_size); // Smoothing core

    size_t centre = core_size / 2 + 1;

    // Core initialization
    for (size_t i = 0; i < core_size; i++) {
        core[i] = Gaussian(i, centre, smooth_factor);
    }

    double core_weight = std::accumulate(core.begin(), core.end(), 0.);

    auto get_smoothed = [&](size_t index) -> cv::Point {
        double x = 0;
        double y = 0;

        for (int i = 0; i < core.size(); i++) {
            int new_index = i - centre + index;
            while (new_index < 0)
                new_index += perimeter.size();
            while (new_index >= perimeter.size())
                new_index -= perimeter.size();

            x += perimeter[new_index].x * core[i];
            y += perimeter[new_index].y * core[i];
        }

        return cv::Point(x / core_weight, y / core_weight);
    };

    //Smooth all point of given perimeter. 
    for (size_t i = 0; i < smoothed_perimeter.size(); i++) {
        smoothed_perimeter[i] = get_smoothed(i);
    }

    smoothed_perimeter = RemoveSuccessiveDuplicates(smoothed_perimeter);
    //smoothed_perimeter = FillGaps(smoothed_perimeter);

    return smoothed_perimeter;
}

std::vector<std::vector<cv::Point>> ImageProc::SmoothPerimeter(const  std::vector<std::vector<cv::Point>>& perimeter, double smooth_factor) {

    std::vector<std::vector<cv::Point>> smoothed_perimeter;

    for (const std::vector<cv::Point>& p : perimeter) {
        smoothed_perimeter.push_back(SmoothPerimeter(p, smooth_factor));
    }

    return smoothed_perimeter;
}

bool ImageProc::PointBelongsToImage(const cv::Mat& image, const cv::Point& p) {
    return p.y >= 0 && p.y < image.rows && p.x >= 0 && p.x < image.cols;
};

bool ImageProc::PointIsMarked(const cv::Mat& image, const cv::Point& p) {
    return PointBelongsToImage(image, p) && image.at<uchar>(p) != 0;
};
