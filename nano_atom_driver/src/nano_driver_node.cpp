#include <nano_atom_driver/nano_driver.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nano_driver");
  ros::NodeHandle n;

  NanoDriver nano_driver(n);
  nano_driver.run();
}
