#ifndef ROBOTIC_85_GRIPPER_UTILS_H
#define ROBOTIC_85_GRIPPER_UTILS_H

#include <iostream>
#include <fstream>
#include <string>

namespace r85_
{

void print_err_msg(const std::string &msg, std::ostream &out = std::cerr)
{
  out << "\033[1m\033[31m" << "[r85_gripper ERROR]: " << msg << "\033[0m";
}

void print_info_msg(const std::string &msg, std::ostream &out = std::cout)
{
  out << "\033[1m\033[34m" << "[r85_gripper INFO]: " << msg << "\033[0m";
}

void print_warn_msg(const std::string &msg, std::ostream &out = std::cout)
{
  out << "\033[1m\033[33m" << "[r85_gripper WARNING]: " << msg << "\033[0m";
}

}; // namespace r85_


#endif // ROBOTIC_85_GRIPPER_UTILS_H
