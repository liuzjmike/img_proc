#include "img_proc.h"

using namespace img_proc;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "img_proc");
  ImageProcessor ip;
  ip.recordFeature(argv[1], argv[2]);
  ros::spin();
  return 0;
}
