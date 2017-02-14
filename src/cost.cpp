#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "opencv2/opencv.hpp"
#include "flann/flann.hpp"

#include <sstream>

#include <img_proc/Point.h>
#include <img_proc/EdgePosition.h>
#include <std_msgs/Float32.h>

using namespace std;
using namespace flann;

class Cost
{
  ros::NodeHandle nh_;
  ros::Subscriber feature_sub_, query_sub_;
  ros::Publisher cost_pub_;

  Matrix<int> f, q;
  vector<img_proc::Point> v;
  boost::mutex feature_mutex;

  int f_count, q_count;

public:
  Cost(char*& feature_topic, char*& query_topic)
    : f_count(0)
    , q_count(0)
  {
    feature_sub_ = nh_.subscribe(feature_topic, 2, &Cost::featureCallback, this);
    query_sub_ = nh_.subscribe(query_topic, 2, &Cost::queryCallback, this);
    cost_pub_ = nh_.advertise<std_msgs::Float32>("cost", 2);
  }

  void featureCallback(const img_proc::EdgePositionConstPtr& msg)
  {
    v.clear();
    v = msg->data;
    f = Matrix<int>(&v[0].x, v.size(), 2);
    cout << "v " << &(v[0].x) << endl;
    cout << "f " << &(f[0][1]) << " " << f[0][1] << endl;
  }

  void queryCallback(const img_proc::EdgePositionConstPtr& msg)
  {
    cout << "q1 " << " " << f[0][1] << endl;
    vector<img_proc::Point> w = msg->data;
    cout << &(msg->data[0]) << " " << &(msg->data[msg->data.size()-1]) << endl;
    cout << &(w[0]) << endl;
    cout << &(w[w.size()-1]) << endl;
    cout << "q2 " << " " << f[0][1] << endl;
    //cout << f[0][1] << endl;
    q = Matrix<int>(&w[0].x, w.size(), 2);
    if(f.rows > 0 && q.rows > 0)
    {
      Matrix<int> features = f;
      #if 0
      for(int i = 0; i < f.rows; i++)
      {
        int x = f[i][0];
        if(x >= 390 && x <= 392)
        {
          cout << x << "\t" << f[i][1] << endl;
        }
      }
      #endif
      Matrix<int> queries = q;
      //computeCost(features, queries);
    }
  }

  void computeCost(const Matrix<int>& features, const Matrix<int>& queries)
  {

    float radius = 64;
    flann::Index<L2<int> > index(features, flann::LinearIndexParams());
    index.buildIndex();
    Matrix<int> indices(new int[queries.rows], queries.rows, 1);
    Matrix<float> dists(new float[queries.rows], queries.rows, 1);
    index.radiusSearch(queries, indices, dists, radius, flann::SearchParams());
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
    std_msgs::Float32 avg_dist;
    avg_dist.data = sum / dists.rows;
    cost_pub_.publish(avg_dist);
  }

  #if 0
  void timer()
  {
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
      boost::mutex::scoped_lock feature_lock(feature_mutex);
      Matrix<int> features = f;
      feature_lock.unlock();
      boost::mutex::scoped_lock query_lock(query_mutex);
      Matrix<int> queries = q;
      query_lock.unlock();
      computeCost(features, queries);
      loop_rate.sleep();
    }
  }
  #endif
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cost");
  Cost c(argv[1], argv[2]);
  //c.timer();
  ros::spin();
  return 0;
}
