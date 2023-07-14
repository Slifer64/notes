#include <main_controller/main_gui.h>
#include <main_controller/utils.h>

using namespace as64_;

std::string getNodeNameID()
{
  std::string id = "";
  ros::NodeHandle("~").getParam("NODE_NAME_ID", id);
  return id;
}

void showMsg(const ExecResultMsg &msg)
{
  emit MainWindow::main_win->showMsgSignal(msg);
}

void showErrorMsg(const std::string &msg)
{
  emit MainWindow::main_win->showMsgSignal(ExecResultMsg(ExecResultMsg::ERROR, msg));
}

void showWarningMsg(const std::string &msg)
{
  emit MainWindow::main_win->showMsgSignal(ExecResultMsg(ExecResultMsg::WARNING, msg));
}

void showQuestionMsg(const std::string &msg)
{
  emit MainWindow::main_win->showMsgSignal(ExecResultMsg(ExecResultMsg::QUESTION, msg));
}

void showInfoMsg(const std::string &msg)
{
  emit MainWindow::main_win->showMsgSignal(ExecResultMsg(ExecResultMsg::INFO, msg));
}

void makeThreadRT(std::thread &thr)
{
  int err_code = thr_::setThreadPriority(thr, SCHED_FIFO, 99);
  if (err_code) PRINT_WARNING_MSG("[MainController::makeThreadRT]: Failed to set thread priority! Reason:\n" + thr_::setThreadPriorErrMsg(err_code) + "\n", std::cerr);
  else PRINT_INFO_MSG("[MainController::makeThreadRT]: Set thread priority successfully!\n", std::cerr);
}

void PRINT_INFO_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[34m" << "[INFO]: " << msg << "\033[0m";
}

void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[32m" << "[INFO]: " << msg << "\033[0m";
}

void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[33m" << "[WARNING]: " << msg << "\033[0m";
}

void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[31m" << "[ERROR]: " << msg << "\033[0m";
}