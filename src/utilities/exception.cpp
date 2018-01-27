/**
 *  This source file implements the Exception helper class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/exception.h"
#include <ros/common.h>

namespace utilities
{
/**
 * @brief Exception::Exception
 * @param message
 */
Exception::Exception(std::string message) : message_(message.c_str())
{
  ROS_FATAL("%s", message.c_str());
}

/**
 * @brief Exception::Exception
 * @param message
 */
Exception::Exception(const char* message) : message_(message)
{
  ROS_FATAL("%s", message);
}

/**
 * @brief Exception::what
 * @return
 */
const char* Exception::what() const throw() { return message_; }
}
