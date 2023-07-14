#ifndef GUI_LIB_SET_POSE_DIALOG_H
#define GUI_LIB_SET_POSE_DIALOG_H

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCloseEvent>

#include <functional>
#include <armadillo>
#include <thread>

#include <gui_lib/utils.h>

namespace as64_
{

namespace gui_
{

class SetPoseDialog : public QDialog
{
  Q_OBJECT

public:
  SetPoseDialog(QWidget *parent = 0);
  ~SetPoseDialog();

  arma::vec getPose() const;
  void setPose(const arma::vec &pose);

signals:
  void poseChanged(bool print_msg=false);

public slots:
  void launch();
  void stop();

private:
  MyLineEdit *x_le;
  MyLineEdit *y_le;
  MyLineEdit *z_le;
  MyLineEdit *qw_le;
  MyLineEdit *qx_le;
  MyLineEdit *qy_le;
  MyLineEdit *qz_le;

  arma::vec pose;

  QPushButton *ok_btn;
  QPushButton *cancel_btn;

  bool run;

  MyLineEdit *createLineEdit();

  void closeEvent(QCloseEvent *event) override;
};

} // namespace gui_

} // namespace as64_

#endif // GUI_LIB_SET_POSE_DIALOG_H
