#include <opencv/cv.h>
#include <opencv/highgui.h>

int main(int argc, const char* argv[])
{
    const cv::Mat input = cv::imread("ferrari.png", 0); //Load as grayscale

    cv::SiftFeatureDetector detector;
    std::vector<cv::KeyPoint> keypoints;
    detector.detect(input, keypoints);

    // Add results to image and save.
    cv::Mat output;
    cv::drawKeypoints(input, keypoints, output);
    cv::imwrite("result.png", output);

    return 0;
}
