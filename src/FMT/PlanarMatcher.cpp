#include <FMT/PlanarMatcher.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

//----------------------------------------------------------------------------//
void hann(cv::Mat &img)
{
    CV_Assert(img.type() == CV_32FC1 || img.type() == CV_64FC1);

    cv::Mat hann_window;
    createHanningWindow(hann_window, img.size(), img.type());
    img = img.mul(hann_window);
}
//----------------------------------------------------------------------------//
void fourier(const cv::Mat &src, cv::Mat &dst)
{
    cv::Mat padded; //expand input image to optimal size
    int m = cv::getOptimalDFTSize(src.rows);
    int n = cv::getOptimalDFTSize(src.cols); // on the border add zero values
    cv::copyMakeBorder(src, padded, 0, m - src.rows, 0, n - src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexI;
    merge(planes, 2, complexI); // Add to the expanded another plane with zeros

    dft(complexI, complexI); // this way the result may fit in the source matrix

    // compute the magnitude and switch to logarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
    split(complexI, planes); // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))

    magnitude(planes[0], planes[1], planes[0]); // planes[0] = magnitude
    dst = planes[0];

    dst += cv::Scalar::all(1); // switch to logarithmic scale
    log(dst, dst);

    // crop the spectrum, if it has an odd number of rows or columns
    dst = dst(cv::Rect(0, 0, dst.cols & -2, dst.rows & -2));

    // rearrange the quadrants of Fourier image  so that the origin is at the image center
    int cx = dst.cols/2;
    int cy = dst.rows/2;

    cv::Mat q0(dst, cv::Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
    cv::Mat q1(dst, cv::Rect(cx, 0, cx, cy));  // Top-Right
    cv::Mat q2(dst, cv::Rect(0, cy, cx, cy));  // Bottom-Left
    cv::Mat q3(dst, cv::Rect(cx, cy, cx, cy)); // Bottom-Right

    cv::Mat tmp; // swap quadrants (Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    q1.copyTo(tmp); // swap quadrant (Top-Right with Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);

    cv::normalize(dst, dst, 0, 1, cv::NORM_MINMAX); // Transform the matrix with float values into a
                                                    // viewable image form (float between values 0 and 1).
}
//----------------------------------------------------------------------------//
cv::Point2f PlanarMatcher::blockMatch(const cv::Mat &block_a, const cv::Mat &block_b)
{
    CV_Assert(block_a.type() == CV_32FC1 && block_b.type() == CV_32FC1);
    CV_Assert(block_a.cols != 0 && block_a.rows != 0 && block_b.cols != 0 && block_b.rows != 0);
    CV_Assert(block_a.cols == block_a.rows && block_b.cols == block_b.rows);
    CV_Assert(block_a.size() == block_b.size());

    //fourier transform
    cv::Mat ft_a, ft_b, lp_a, lp_b;
    fourier(block_a, ft_a);
    fourier(block_b, ft_b);

    //log-polar transform
    cv::Point2f center(ft_a.size() / 2);
    double radius = (double)ft_a.cols / 4;
    double M = (double)ft_a.cols / log(radius);
    cv::logPolar(ft_a, lp_a, center, M, cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);
    cv::logPolar(ft_b, lp_b, center, M, cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);

    //log-polars phase correlation
    cv::Point2f shift = phaseCorrelate(lp_b, lp_a);

    //angle
    double angle = -shift.y * 360.0 / block_a.rows;
    if(std::abs(angle) > 90)
        angle += 180.0;

    //scale factor
    double scale = 1.0 / exp(shift.x/M);
    cv::Mat transform_mat = getRotationMatrix2D(center, angle, scale);

    //rotation/scale precomp
    cv::Mat block_b_rs;
    warpAffine(block_b, block_b_rs, transform_mat, block_b.size());

    //final phase correlation
    return phaseCorrelate(block_a, block_b_rs);
}
//----------------------------------------------------------------------------//
//Return empty matrix if no homography found
cv::Mat PlanarMatcher::imageMatch(const cv::Mat &img_a, const cv::Mat &img_b)
{
    std::vector<cv::Point2f> srcPoints, dstPoints;
    multiBlockMatch(img_a, img_b, srcPoints, dstPoints);

    cv::Mat homography = findHomography(srcPoints, dstPoints, cv::LMEDS);
    if(homography.data == NULL)
        throw std::logic_error("Error : PlanarMatcher homography calculation failed.");

    return homography;
}
//----------------------------------------------------------------------------//
void PlanarMatcher::multiBlockMatch(const cv::Mat &img_a, const cv::Mat &img_b, std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints)
{
    CV_Assert(img_a.type() == CV_32FC1 && img_b.type() == CV_32FC1);
    CV_Assert(img_a.cols != 0 && img_a.rows != 0 && img_b.cols != 0 && img_b.rows != 0);
    CV_Assert(img_a.size() == img_b.size());

    int block_size = 240;

    cv::Mat a_copy = img_a.clone();
    cv::Mat b_copy = img_b.clone();

    for(int i=0;i<img_a.cols / block_size;i++)
    {
        for(int j=0;j<img_a.rows / block_size;j++)
        {
            cv::Rect roi(i*block_size, j*block_size, block_size, block_size);
            cv::Mat block_a = a_copy(roi);
            cv::Mat block_b = b_copy(roi);

            hann(block_a);
            hann(block_b);

            cv::Point2f tr = blockMatch(block_a, block_b);

            srcPoints.push_back(cv::Point2f(i*block_size + block_size/2, j*block_size + block_size/2));
            dstPoints.push_back(cv::Point2f(i*block_size + block_size/2 - tr.x, j*block_size + block_size/2 - tr.y));
        }
    }
}
//----------------------------------------------------------------------------//
