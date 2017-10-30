#include <FMT/PlanarMatcher.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <unistd.h>
#include <iostream>

bool update = true;
//----------------------------------------------------------------------------//
void hann_fourier(const cv::Mat &src, cv::Mat &dst)
{
    src.convertTo(dst, CV_32FC1);
    hann(dst);
    fourier(dst, dst);
}
//----------------------------------------------------------------------------//
void transform(cv::Mat &img, int x, int y, int r, float s)
{
    cv::Mat M = (cv::Mat_<double>(2,3) << 1, 0, x, 0, 1, y);
    cv::warpAffine(img, img, M, img.size());

    M = getRotationMatrix2D(cv::Point2f(img.cols/2, img.rows/2), 0, s);
    cv::warpAffine(img, img, M, img.size());

    M = getRotationMatrix2D(cv::Point2f(img.cols/2, img.rows/2), r, 1);
    cv::warpAffine(img, img, M, img.size());
}
//----------------------------------------------------------------------------//
void on_trackbar( int, void* )
{
    update = true;
}
//----------------------------------------------------------------------------//
void pipeline(cv::Mat &img, cv::Mat &dst,  int x, int y, int r, float s)
{
    transform(img, x, y, r, s);

    hann_fourier(img, dst);

    cv::normalize(dst, dst, 0, 1, cv::NORM_MINMAX, CV_32FC1);

    cv::imshow("FFT", dst);

    cv::Mat img_polar;
    cv::Point2f center(dst.size() / 2);
    double radius = (double)dst.cols / 4;
    double M = (double)dst.cols / log(radius);
    cv::linearPolar(dst, img_polar, center, M, cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);

    cv::logPolar(dst, dst, center, M, cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);

    cv::normalize(dst, dst, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    cv::imshow("input", img);
    cv::imshow("Polar", img_polar);
    cv::imshow("LogPolar", dst);
}
//----------------------------------------------------------------------------//
int main(int argc, char **argv)
{
    if(argc < 2)
    {
        std::cout << "need one image."<< std::endl;
        return -1;
    }

    cv::Mat img;
    img = imread(argv[1], cv::IMREAD_ANYDEPTH);
    int min_side = std::min(img.cols, img.rows);
    img = img(cv::Rect(0, 0, min_side, min_side));

    cv::normalize(img, img, 0, 65535, cv::NORM_MINMAX, CV_16UC1);

    cv::namedWindow("input", CV_WINDOW_NORMAL);
    cv::resizeWindow("input", 620, 620);
    cv::moveWindow("input", 0, 0);
    cv::namedWindow("FFT", CV_WINDOW_NORMAL);
    cv::resizeWindow("FFT", 620, 620);
    cv::moveWindow("FFT", 640, 0);
    cv::namedWindow("LogPolar", CV_WINDOW_NORMAL);
    cv::resizeWindow("LogPolar", 620, 620);
    cv::moveWindow("LogPolar", 1280, 0);
    cv::namedWindow("Polar", CV_WINDOW_NORMAL);
    cv::resizeWindow("Polar", 620, 620);
    cv::moveWindow("Polar", 1280, 0);

    cv::namedWindow("commands", CV_WINDOW_NORMAL);
    cv::moveWindow("commands", 1920/2-150, 700);
    int translation_x;
    cv::createTrackbar("translation x", "commands", &translation_x, 200, &on_trackbar);
    cv::setTrackbarPos("translation x", "commands", 100);

    int translation_y;
    cv::createTrackbar("translation y", "commands", &translation_y, 200, &on_trackbar);
    cv::setTrackbarPos("translation y", "commands", 100);

    int rotation;
    cv::createTrackbar("rotation", "commands", &rotation, 180, &on_trackbar);
    cv::setTrackbarPos("rotation", "commands", 90);

    int scale;
    cv::createTrackbar("scale", "commands", &scale, 100, &on_trackbar);
    cv::setTrackbarPos("scale", "commands", 50);

    while (true)
    {
        if(update)
        {
            cv::Mat img_mod = img.clone();
            cv::Mat output;
            pipeline(img_mod, output, translation_x-100, translation_y-100, rotation-90, float(scale) / 50);
        }
        update = false;
        if(cv::waitKey(1) == 27)
            exit(0);
    }

    return 0;
}
//----------------------------------------------------------------------------//
