#ifndef _CLI_H_
#define _CLI_H_

#include <opencv2/core/core.hpp>
#include <vector>

template<class T>
using Vec3D = std::vector<std::vector<std::vector<T>>>;

/*
Displays image in a new window.
params: win_name - name of the new window.
        image - image to be displayed.
*/
void DisplayImage(const std::string& win_name, const cv::Mat& image);

/*
Writes regions and perimeters data to a file.
params: regions - vector of regions to be written
        perimeters - vector of perimeters to be written
        file_name - name of the file
*/
void Store(const std::vector<cv::Mat>& regions, const Vec3D<cv::Point>& perimeters, const std::string& file_name);

void NewRegionCommand(const cv::Mat& image, std::vector<cv::Mat>& regions, Vec3D<cv::Point>& perimeters);

void DisplayCommand(cv::Mat& image, const std::vector<cv::Mat>& regions, const Vec3D<cv::Point>& perimeters);

void CleanCommand(const cv::Mat& image, cv::Mat& displayed_image, std::vector<cv::Mat>& regions, Vec3D<cv::Point>& perimeters);

void StoreCommand(const std::vector<cv::Mat>& regions, const Vec3D<cv::Point>& perimeters);

void CommandLoop(const cv::Mat& image);

#endif //_CLI_H_
