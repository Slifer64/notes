#ifndef GUI_APPLICATION_H
#define GUI_APPLICATION_H

#include <QApplication>
#include <exception>
#include <iostream>

namespace as64_
{

namespace gui_
{

class GuiApplication : public QApplication
{
Q_OBJECT

public:
  GuiApplication(int& argc, char** argv) :
  QApplication(argc, argv) {}

  bool notify(QObject* receiver, QEvent* event) 
  {
    bool done = true;
    try 
    {
      done = QApplication::notify(receiver, event);
    } 
    catch (const std::exception& e) 
    {
      std::cerr << "\033[1m" << "\033[31m" << "[QApplication::exception]: " << e.what() << "\033[0m\n";
    }
    return done;
  } 

};

} // namespace gui_

} // namespace as64_

#endif // GUI_APPLICATION_H
