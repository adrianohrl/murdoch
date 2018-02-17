#include "murdoch/task_generator_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_generator_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle());
  murdoch::TaskGeneratorNodePtr node(new murdoch::TaskGeneratorNode(nh));
  node->run();
  return EXIT_SUCCESS;
}
