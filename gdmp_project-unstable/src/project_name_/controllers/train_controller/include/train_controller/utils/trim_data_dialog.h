#ifndef TRAIN_CONTROLLER_GUI_TRIM_DATA_DIALOG_H
#define TRAIN_CONTROLLER_GUI_TRIM_DATA_DIALOG_H

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


class TrimDataDialog : public QDialog
{
Q_OBJECT

public:
  TrimDataDialog(QWidget *parent = 0);
  ~TrimDataDialog();

  void lauch() { if (!this->isVisible()) this->show(); }

  double getVelTrimThres() const { return vel_trim_le->text().toDouble(); }
  double getRotVelTrimThres() const { return rotVel_trim_le->text().toDouble(); }

signals:
  void applyTrimSignal();


private:

  QPushButton *apply_trim_btn;
  QLineEdit *vel_trim_le;
  QLineEdit *rotVel_trim_le;

};

#endif // TRAIN_CONTROLLER_GUI_TRIM_DATA_DIALOG_H
