#include "orb_matcher.h"
#include "utils.h"

int main(int argc, char **argv)
{
    if (argc != 5)
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

    // std::vector<std::vector<float>> depth_gt;
    // readDepthImage(argv[3], depth_gt);
    // if(depth_gt.empty())
    // {
    //     std::cerr << "Error: Could not open or find the image 'depth_gt.png'" << std::endl;
    //     return -1;
    // }

    std::vector<std::vector<float>> depth_pred_1;
    if (!readTiffImage(argv[3], depth_pred_1))
    {
        std::cerr << "Error: Could not open or find the image 'depth_pred_1.tiff'" << std::endl;
        return -1;
    }

    std::vector<std::vector<float>> depth_pred_2;
    if (!readTiffImage(argv[4], depth_pred_2))
    {
        std::cerr << "Error: Could not open or find the image 'depth_pred_2.tiff'" << std::endl;
        return -1;
    }

    ORBMatcher orb_matcher;
    std::pair<std::vector<cv::Point>, std::vector<cv::Point>> pixel_cords = orb_matcher.match(img_1, img_2);


    // orb_matcher.visualize_matches(img_1, img_2, pixel_cords.first, pixel_cords.second, "Matches");
    // cv::waitKey(0);
    return 0;
}