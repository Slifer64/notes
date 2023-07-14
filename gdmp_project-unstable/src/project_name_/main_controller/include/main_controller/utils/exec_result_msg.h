#ifndef $_PROJECT_384$_EXEC_MSG_RESULT_H
#define $_PROJECT_384$_EXEC_MSG_RESULT_H

#include <string>
#include <QMetaType>
#include <QDebug>

class ExecResultMsg
{
public:

  enum TYPE
  {
      QUESTION = 1,
      INFO = 0,
      WARNING = -1,
      ERROR = -2
  };

  ExecResultMsg(ExecResultMsg::TYPE type=ExecResultMsg::INFO, const std::string &msg="") { this->type=type; this->msg=msg; }

  bool empty() const { return msg.empty(); }
  std::string getMsg() const { return msg; }
  void setMsg(const std::string &msg) { this->msg = msg; }
  void addToMsg(const std::string &msg) { this->msg += msg; }
  void setType(ExecResultMsg::TYPE type) { this->type = type; }
  ExecResultMsg::TYPE getType() const { return type; }
private:
  std::string msg;
  ExecResultMsg::TYPE type;
};

Q_DECLARE_METATYPE(ExecResultMsg);


#endif // $_PROJECT_384$_EXEC_MSG_RESULT_H
