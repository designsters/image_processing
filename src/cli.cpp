#include "cli.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <functional>
#include <map>
#include "imageproc.h"

const std::string help_string = "Supported commands:\n"\
                                "    region <x> <y> - add new region\n"
                                "    display - show regions and perimeters in a window\n"
                                "    clean - delete all regions\n"
                                "    smooth <factor> - smooth perimeters\n"
                                "    store <file name> - store regions and perimeters data into a file\n"
                                "    help - show a list of commands\n"
                                "    exit - terminate the program\n";

void DisplayImage(const std::string& win_name, const cv::Mat& image) {
    cv::namedWindow(win_name);
    cv::imshow(win_name, image);
    cv::startWindowThread(); // "This window is not responding" workaround.

    cv::waitKey(0);
    cv::destroyWindow(win_name);
}

void Store(const std::vector<cv::Mat>& regions, const Vec3D<cv::Point>& perimeters, const std::string& file_name) {
    std::ofstream out(file_name);

    //Write regions to the file
    for (size_t i = 0; i < regions.size(); i++) {
        out << "region " << i << '\n';
        for (auto it = regions[i].begin<uchar>(); it != regions[i].end<uchar>(); it++) {
            if (*it != 0) {
                out << it.pos() << ' ';
            }
        }
        out << '\n';
    }

    //Write perimeters to the file
    for (size_t i = 0; i < perimeters.size(); i++) {
        out << "perimeter " << i << '\n';
        for (const auto& vect : perimeters[i]) {
            for (const auto& point : vect) {
                out << point << ' ';
            }
        }
        out << '\n';
    }
}

void NewRegionCommand(const cv::Mat& image, std::vector<cv::Mat>& regions, Vec3D<cv::Point>& perimeters) {
    int x, y;
    std::cin >> x >> y;

    auto region = ImageProc::FindRegion(image, cv::Point(x, y), cv::Vec3b(50, 50, 50), cv::Vec3b(5, 5, 5));
    auto perimeter = ImageProc::FindPerimeter(region);
    regions.push_back(region);
    perimeters.push_back(perimeter);
}

void DisplayCommand(cv::Mat& image, const std::vector<cv::Mat>& regions, const Vec3D<cv::Point>& perimeters) {
    const std::string win_name = "Image";

    for (const auto& i : regions)
        ImageProc::DumpPixels(image, i, cv::Vec3b(255, 255, 255));
    for (const auto& i : perimeters)
        ImageProc::DumpPixels(image, i, cv::Vec3b(0, 0, 255));
    DisplayImage(win_name, image);
}

void SmoothCommand(Vec3D<cv::Point>& perimeters) {
    double factor;
    std::cin >> factor;

    for (auto& i : perimeters) {
        i = ImageProc::SmoothPerimeter(i, factor);
    }
}

void CleanCommand(const cv::Mat& image, cv::Mat& displayed_image, std::vector<cv::Mat>& regions, Vec3D<cv::Point>& perimeters) {
    regions.clear();
    perimeters.clear();
    image.copyTo(displayed_image);
}

void StoreCommand(const std::vector<cv::Mat>& regions, const Vec3D<cv::Point>& perimeters) {
    std::string file_name;
    std::cin >> file_name;
    Store(regions, perimeters, file_name);
}

void CommandLoop(const cv::Mat& image) {
    bool terminate = false;
    std::vector<cv::Mat> regions;
    Vec3D<cv::Point> perimeters;
    cv::Mat displayed_image;
    image.copyTo(displayed_image);

    std::map<std::string, std::function<void()>> command_map; // map<command name, function>

    using std::ref;
    using std::cref;
    command_map["region"]  = std::bind(NewRegionCommand, cref(image), ref(regions), ref(perimeters));
    command_map["display"] = std::bind(DisplayCommand, ref(displayed_image), cref(regions), cref(perimeters));
    command_map["smooth"]  = std::bind(SmoothCommand, ref(perimeters));
    command_map["clean"]   = std::bind(CleanCommand, cref(image), ref(displayed_image), ref(regions), ref(perimeters));
    command_map["store"]   = std::bind(StoreCommand, cref(regions), cref(perimeters));
    command_map["help"]    = []() { std::cout << help_string << '\n'; };
    command_map["exit"]    = [&terminate]() { terminate = true; };

    while (!terminate) {
        std::cout << '>';
        std::string command;
        std::cin >> command;

        if (command_map.find(command) != command_map.end()) {
            command_map.at(command)();
        }
        else {
            std::cout << "Command \"" << command << "\" does not exist. Type \"help\" for a list of commands.\n";
        }
    }
}
