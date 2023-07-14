#ifndef GUI_LIB_VIEW_PROGRESS_DIALOG_H
#define GUI_LIB_VIEW_PROGRESS_DIALOG_H

#include <QDialog>
#include <QLabel>
#include <QVBoxLayout>
#include <QCloseEvent>
#include <QProgressBar>

#include <functional>
#include <thread>

#include <gui_lib/utils.h>

namespace as64_
{

namespace gui_
{

class ViewProgressDialog : public QDialog
{
  Q_OBJECT

public:
  enum OperationType
  {
    FORWARD_OP,
    REVERSE_OP
  };

  ViewProgressDialog(std::function<double()> getProgressValue, QWidget *parent = 0);
  ~ViewProgressDialog();

signals:
  void setOperationTypeSignal(enum OperationType op_type);

public slots:
  void launch();
  void stop();

  void startProgressBarUpdate();
  void stopProgressBarUpdate();
  void setOperationType(enum OperationType op_type);

private:
  std::function<double()> get_progress_value;

  gui_::Semaphore start_sem;
  bool update_prog_bar;

  QProgressBar *prog_bar;
  QLabel *prog_lb;

  bool run;
  void closeEvent(QCloseEvent *event) override;
};

} // namespace gui_

} // namespace as64_

#endif // GUI_LIB_VIEW_PROGRESS_DIALOG_H
