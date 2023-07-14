#ifndef TRAIN_CONTROLLER_GUI_REMOVE_STOPS_DIALOG_H
#define TRAIN_CONTROLLER_GUI_REMOVE_STOPS_DIALOG_H

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QSlider>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCloseEvent>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>

#include <vector>
#include <cstring>
#include <armadillo>
#include <functional>
#include <thread>
#include <chrono>


class RemoveStopsDialog : public QDialog
{
Q_OBJECT

public:
  RemoveStopsDialog(QWidget *parent = 0);
  ~RemoveStopsDialog();

  void lauch() { if (!this->isVisible()) this->show(); }

  double getVelTrimThres() const { return vel_thres_le->text().toDouble(); }

signals:
  void removeStopsSignal();


private:

  QPushButton *remove_stops_btn;
  QLineEdit *vel_thres_le;

};

#endif // TRAIN_CONTROLLER_GUI_REMOVE_STOPS_DIALOG_H
