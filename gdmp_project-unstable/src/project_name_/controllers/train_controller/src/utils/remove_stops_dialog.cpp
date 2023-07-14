#include <train_controller/utils/remove_stops_dialog.h>

#include <iostream>

RemoveStopsDialog::RemoveStopsDialog(QWidget *parent): QDialog(parent)
{
  this->setWindowTitle("Remove stops from demo data");

  QFont font1 = QFont("Ubuntu", 15, QFont::DemiBold);
  QFont font2 = QFont("Ubuntu", 13, QFont::DemiBold);

  remove_stops_btn = new QPushButton("Apply");
  remove_stops_btn->setFont(font2);
  QObject::connect( remove_stops_btn, &QPushButton::pressed, [this](){ emit this->removeStopsSignal(); });

  QLabel *vel_thres_lb = new QLabel("velocity threshold");
  vel_thres_lb->setFont(font2);
  vel_thres_le = new QLineEdit("0.0001");
  vel_thres_le->setFont(font2);
  vel_thres_le->setAlignment(Qt::AlignCenter);

  QGridLayout *vel_thres_layout = new QGridLayout;
  vel_thres_layout->addWidget(vel_thres_lb, 0,0);
  vel_thres_layout->addWidget(vel_thres_le, 0,1);

  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->addLayout(vel_thres_layout);
  main_layout->addWidget(remove_stops_btn, Qt::AlignRight);
  main_layout->addStretch(0);

}

RemoveStopsDialog::~RemoveStopsDialog()
{

}
