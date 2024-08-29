#include "orb_matcher.h"

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cout << "usage: feature_extraction img1 img2" << std::endl;
        return 1;
    }

    /* Read image. */
    cv::Mat img_1 = imread(argv[1], cv::IMREAD_COLOR);
    if (img_1.empty())
    {
        std::cerr << "Error: Could not open or find the image '1.png'" << std::endl;
        return -1;
    }

    cv::Mat img_2 = imread(argv[2], cv::IMREAD_COLOR);
    if (img_2.empty())
    {
        std::cerr << "Error: Could not open or find the image '2.png'" << std::endl;
        return -1;
    }

    ORBMatcher orb_matcher;
    orb_matcher.match(img_1, img_2);

    return 0;
}