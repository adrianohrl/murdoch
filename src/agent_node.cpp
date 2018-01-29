#include "murdoch/agent_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "auctioneer_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle());
  murdoch::AgentNodePtr node(new murdoch::AgentNode(nh));
  node->run();
  return EXIT_SUCCESS;
}
