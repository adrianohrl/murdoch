#include "murdoch/bidder_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bidder_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle());
  murdoch::BidderNodePtr node(new murdoch::BidderNode(nh));
  node->run();
  return EXIT_SUCCESS;
}
