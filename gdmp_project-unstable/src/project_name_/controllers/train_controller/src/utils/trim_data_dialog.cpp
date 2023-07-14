#include <train_controller/utils/trim_data_dialog.h>

#include <iostream>

TrimDataDialog::TrimDataDialog(QWidget *parent): QDialog(parent)
{
  this->setWindowTitle("Trim recorded data");

  QFont font1 = QFont("Ubuntu", 15, QFont::DemiBold);
  QFont font2 = QFont("Ubuntu", 13, QFont::DemiBold);

  apply_trim_btn = new QPushButton("Apply");
  apply_trim_btn->setFont(font2);
  QObject::connect( apply_trim_btn, &QPushButton::pressed, [this](){ emit this->applyTrimSignal(); });

  QLabel *vel_trim_lb = new QLabel("pos_trim_thres");
  vel_trim_lb->setFont(font2);
  vel_trim_le = new QLineEdit("0");
  vel_trim_le->setFont(font2);
  vel_trim_le->setAlignment(Qt::AlignCenter);

  QLabel *rotVel_trim_lb = new QLabel("orient_trim_thres");
  rotVel_trim_lb->setFont(font2);
  rotVel_trim_le = new QLineEdit("0");
  rotVel_trim_le->setFont(font2);
  rotVel_trim_le->setAlignment(Qt::AlignCenter);

  QGridLayout *trim_thres_layout = new QGridLayout;
  trim_thres_layout->addWidget(vel_trim_lb, 0,0);
  trim_thres_layout->addWidget(vel_trim_le, 0,1);
  trim_thres_layout->addWidget(rotVel_trim_lb, 1,0);
  trim_thres_layout->addWidget(rotVel_trim_le, 1,1);

  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->addLayout(trim_thres_layout);
  main_layout->addWidget(apply_trim_btn, Qt::AlignRight);
  main_layout->addStretch(0);

}

TrimDataDialog::~TrimDataDialog()
{

}
