#include <global_gui/global_gui.h>

#include <iostream>
#include <memory>
#include <csignal>
#include <ros/ros.h>

#include <QApplication>

GlobalWindow *gui;

void closeGUI(int)
{
  emit gui->closeSignal();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Global GUI app");

  QApplication app(argc, argv);
  QThread::currentThread()->setPriority(QThread::LowestPriority);

  gui = new GlobalWindow;
  gui->show();

  signal(SIGINT, closeGUI);

  app.exec();

  delete gui; // must be destructed in this thread!

  return 0;
}
