#include "murdoch/task_executor_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_executor_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle());
  murdoch::TaskExecutorNodePtr node(new murdoch::TaskExecutorNode(nh));
  node->run();
  return EXIT_SUCCESS;
}
