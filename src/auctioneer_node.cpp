#include "murdoch/auctioneer_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "auctioneer_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle());
  murdoch::AuctioneerNodePtr node(new murdoch::AuctioneerNode(nh));
  node->run();
  return EXIT_SUCCESS;
}
