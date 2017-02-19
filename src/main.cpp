#include "img_proc.h"

using namespace img_proc;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "img_proc");
  ImageProcessor ip;
  ip.listenFeature(argv[1], argv[2]);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
