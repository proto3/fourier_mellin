#ifndef PLANAR_MATCHER_H
#define PLANAR_MATCHER_H
#include <opencv2/core.hpp>

void fourier(const cv::Mat &src, cv::Mat &dst);
void hann(cv::Mat &img);

class PlanarMatcher
{
public:
    static cv::Point2f blockMatch(const cv::Mat &block_a, const cv::Mat &block_b);
    static void multiBlockMatch(const cv::Mat &img_a, const cv::Mat &img_b, std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints);
    static cv::Mat imageMatch(const cv::Mat &img_a, const cv::Mat &img_b);
};

#endif
