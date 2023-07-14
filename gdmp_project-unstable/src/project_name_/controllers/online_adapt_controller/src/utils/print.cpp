
#include <online_adapt_controller/utils/print.h>

namespace online_adapt_
{

void PRINT_INFO_MSG(const std::string &msg)
{ std::cerr << "\033[1m\033[34m" << msg << "\033[0m" << std::flush; }

void PRINT_SUCCESS_MSG(const std::string &msg)
{ std::cerr << "\033[1m\033[32m" << msg << "\033[0m" << std::flush; }

void PRINT_WARNING_MSG(const std::string &msg)
{ std::cerr << "\033[1m\033[33m" << msg << "\033[0m" << std::flush; }

void PRINT_ERROR_MSG(const std::string &msg)
{ std::cerr << "\033[1m\033[31m" << msg << "\033[0m" << std::flush; }


} // namespace online_adapt_

