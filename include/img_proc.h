#include "opencv2/opencv.hpp"
#include "flann/flann.hpp"

#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H

class ImageProcessor
{
public:
  ImageProcessor();

  static cv::Mat detectEdge(const cv::Mat& image, const int loThresh=100, const int hiThresh=160);
  static std::vector<int> locateEdge(const cv::Mat& edgeImage);
  static flann::Index<flann::L2<int> > buildIndex(std::vector<int> features, const flann::IndexParams& params);
  static float diff(const flann::Index<flann::L2<int> >& index, std::vector<int> queries, const float radius);
};

#endif
