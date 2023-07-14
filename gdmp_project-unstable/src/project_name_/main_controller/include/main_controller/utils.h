#ifndef $_PROJECT_384$_UTILS_H
#define $_PROJECT_384$_UTILS_H

#define PROJECT_NAME "main_controller"

#include <ros/ros.h>

#include <thread_lib/thread_lib.h>

#include <main_controller/utils/exec_result_msg.h>
#include <main_controller/utils/timer.h>

using namespace as64_;

std::string getNodeNameID();

void showMsg(const ExecResultMsg &msg);

void showErrorMsg(const std::string &msg);

void showWarningMsg(const std::string &msg);

void showQuestionMsg(const std::string &msg);

void showInfoMsg(const std::string &msg);

void makeThreadRT(std::thread &thr);

void PRINT_INFO_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out = std::cout);


#include <string>
#include <sstream>
#include <vector>
#include <armadillo>
template<typename T>
std::string vec2str(const T &v, int precision=6, const std::string &separator=", ")
{
  std::ostringstream oss;
  oss.precision(precision);

  for (int i=0; i<v.size()-1; i++) oss << v[i] << separator;
  oss << v[v.size()-1];

  return oss.str();
}

#endif // $_PROJECT_384$_UTILS_H
