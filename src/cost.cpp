#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "opencv2/opencv.hpp"
#include "flann/flann.hpp"

#include <stdio.h>

#include <img_proc/Point.h>
#include <img_proc/EdgePosition.h>

using namespace std;
using namespace flann;

class Cost
{
  ros::NodeHandle nh_;
  ros::Subscriber edge_sub1_, edge_sub2_;

  vector<img_proc::Point> features, queries;
  boost::mutex feature_mutex, query_mutex;

public:
  Cost(char*& sub_topic1, char*& sub_topic2)
  {
    edge_sub1_ = nh_.subscribe(sub_topic1, 2, &Cost::topic1Callback, this);
    edge_sub2_ = nh_.subscribe(sub_topic2, 2, &Cost::topic2Callback, this);
  }

  void topic1Callback(const img_proc::EdgePositionConstPtr& msg)
  {
    boost::mutex::scoped_lock feature_lock(feature_mutex);
    features = msg->data;
  }

  void topic2Callback(const img_proc::EdgePositionConstPtr& msg)
  {
    boost::mutex::scoped_lock query_lock(query_mutex);
    queries = msg->data;
  }

  void computeCost()
  {
    int nn = 1;
    img_proc::Point p, q;
    p.x = 0;
    p.y = 0;
    q.x = 1;
    q.y = 3;
    vector<img_proc::Point> features, queries;
    features.push_back(p);
    queries.push_back(q);
    flann::Index<L2<int> > index(Matrix<int>(&features[0].x, features.size(), 2), flann::KDTreeIndexParams(4));
    index.buildIndex();
    Matrix<int> indices(new int[queries.size()*nn], queries.size(), nn);
    Matrix<float> dists(new float[queries.size()*nn], queries.size(), nn);
    index.knnSearch(Matrix<int>(&queries[0].x, queries.size(), 2), indices, dists, nn, flann::SearchParams());
    for (int i = 0 ; i < indices.rows ; ++i)
    {
        for (int j = 0 ; j < indices.cols ; ++j)
        {
            cout << dists[i][j] << "\t" ;
        }
        cout << endl ;
    }
  }

  void timer()
  {
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
      boost::mutex::scoped_lock feature_lock(feature_mutex);
      feature_lock.unlock();
      boost::mutex::scoped_lock query_lock(query_mutex);
      cout << ros::Time::now() << endl;
      loop_rate.sleep();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cost");
  Cost c(argv[1], argv[2]);
  c.computeCost();
  c.timer();
  //ros::spin();
  return 0;
}
