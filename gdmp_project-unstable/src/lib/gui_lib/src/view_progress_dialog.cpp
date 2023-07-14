#include <gui_lib/view_progress_dialog.h>

namespace as64_
{

namespace gui_
{

ViewProgressDialog::ViewProgressDialog(std::function<double()> getProgressValue, QWidget *parent): QDialog(parent)
{
  run = false;

  this->resize(450,100);

  get_progress_value = getProgressValue;

  QFont font1 = QFont("Ubuntu", 17, QFont::DemiBold);
  QFont font2 = QFont("Ubuntu", 15, QFont::DemiBold);
  QFont font3 = QFont("Ubuntu", 14, 57);
  QFont font4 = QFont("Ubuntu", 12, QFont::Normal);

  // --------------------------------------------------

  prog_bar = new QProgressBar;
  prog_bar->setOrientation(Qt::Horizontal);
  prog_bar->setRange(0,10000);
  prog_bar->setFont(font2);
  //prog_bar->setAlignment(Qt::Alignment alignment)
  prog_bar->setFormat("%p");


  prog_lb = new QLabel("");
  prog_lb->setFont(font2);

  QVBoxLayout *progress_layout = new QVBoxLayout(this);
  progress_layout->addWidget(prog_lb);
  progress_layout->addWidget(prog_bar);
  progress_layout->addStretch(0);

  QObject::connect(this, &ViewProgressDialog::setOperationTypeSignal, this, &ViewProgressDialog::setOperationType);
  QObject::connect(prog_bar, &QProgressBar::valueChanged, prog_bar, &QProgressBar::setValue);

}

ViewProgressDialog::~ViewProgressDialog()
{
  stop();
}

void ViewProgressDialog::launch()
{
  if (!run)
  {
    run = true;
    this->show();

    std::thread([this]()
    {
      while (run)
      {
        update_prog_bar = false;
        start_sem.wait();
        update_prog_bar = true;
        while (update_prog_bar && run)
        {
          emit prog_bar->valueChanged(get_progress_value()*10000);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    }).detach();

  }
}

void ViewProgressDialog::stop()
{
  if (run)
  {
    start_sem.notify(); // to avoid possibly deadlocks
    run = false;
    this->hide();
  }
}

void ViewProgressDialog::setOperationType(enum OperationType op_type)
{
  QString text;
  QString style_sheet;

  if (op_type == FORWARD_OP)
  {
    text = "FORWARD";
    style_sheet = "background-color: rgb(245,245,245); color: rgb(0,200,0); font: 75 14pt \"FreeSans\";";
  }
  else if (op_type == REVERSE_OP)
  {
    text = "REVERSE";
    style_sheet = "background-color: rgb(245,245,245); color: rgb(220, 0, 0); font: 75 14pt \"FreeSans\";";
  }

  prog_lb->setText(text);
  prog_lb->setStyleSheet(style_sheet);
}

void ViewProgressDialog::startProgressBarUpdate()
{
  if (run) start_sem.notify();
}

void ViewProgressDialog::stopProgressBarUpdate()
{
  update_prog_bar = false;
  prog_lb->setText("");
  prog_bar->reset();
}

void ViewProgressDialog::closeEvent(QCloseEvent *event)
{
  stop();
}

} // namespace gui_

} // namespace as64_
