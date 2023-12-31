#ifndef ROBOT_CAMERA_TF_PUBLISHER_PRINT_UTILS_H
#define ROBOT_CAMERA_TF_PUBLISHER_PRINT_UTILS_H

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <string>
#include <iomanip>

void PRINT_INFO_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out = std::cout);

#endif // ROBOT_CAMERA_TF_PUBLISHER_PRINT_UTILS_H
