#include "img_proc.h"

using std::vector;
using cv::Mat;
using cv::Size;
using namespace flann;

ImageProcessor::ImageProcessor()
{}

Mat ImageProcessor::detectEdge(const Mat& image, const int loThresh, const int hiThresh)
{
  Mat gray, blurImg;
  cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  blur(gray, blurImg, Size(3, 3));
  Canny(blurImg, gray, loThresh, hiThresh);
  return gray;
}

vector<int> ImageProcessor::locateEdge(const Mat& edgeImage)
{
  vector<int> edgePoints;
  for(int r = 0; r < edgeImage.rows; r++)
  {
    for(int c = 0; c < edgeImage.cols; c++)
    {
      if(edgeImage.at<uchar>(r, c) > 127)
      {
        edgePoints.push_back(c);
        edgePoints.push_back(r);
      }
    }
  }
  return edgePoints;
}

Index<L2<int> > ImageProcessor::buildIndex(vector<int> features, const IndexParams& params)
{
  Index<L2<int> > index(Matrix<int>(&features[0], features.size()/2, 2), params);
  index.buildIndex();
  return index;
}

float ImageProcessor::diff(const Index<L2<int> >& index, vector<int> queries, const float radius)
{
  Matrix<int> q(&queries[0], queries.size()/2, 2);
  Matrix<int> indices(new int[q.rows], q.rows, 1);
  Matrix<float> dists(new float[q.rows], q.rows, 1);
  index.radiusSearch(q, indices, dists, radius, SearchParams());
  float sum = 0;
  for(int i = 0; i < dists.rows; i++)
  {
    float d = dists[i][0];
    if(d > radius * radius)
    {
      sum += radius * radius * 2;
      #if 0
      cout << queries[i][0] << "\t" << queries[i][1] << "\t" << d << endl;
      for(int j = 0; j < features.rows; j++)
      {
        if(features[j][0] == queries[i][0])
        {
          cout << features[j][0] << "\t" << features[j][1] << endl;
        }
      }
      //int ind = indices[i][0];
      //cout << queries[i][0] << "\t" << queries[i][1] << endl;
      #endif
    }
    else
    {
      sum += d;
    }
  }
  return sum / dists.rows;
}
