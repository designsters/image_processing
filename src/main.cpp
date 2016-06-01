#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "cli.h"

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cout << " Usage: ImageProc.out <image path>\n";
        return EXIT_FAILURE;
    }

    cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR); //Read the image.

    if (!image.data) {
        std::cout << "Could not open or find the image\n";
        return EXIT_FAILURE;
    }

    CommandLoop(image);

    return EXIT_SUCCESS;
}
