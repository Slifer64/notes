#include <gui_lib/set_joints_pos_dialog.h>

#include <QDebug>

// ======================================================
// ===============    SetJointsPosDialog    ==================
// ======================================================

namespace as64_
{

namespace gui_
{

SetJointsPosDialog::SetJointsPosDialog(const std::function<arma::vec()> &get_joints_pos_fun, QWidget *parent): QDialog(parent)
{
  init(get_joints_pos_fun);
}

SetJointsPosDialog::SetJointsPosDialog(const std::function<arma::vec()> &get_joints_pos_fun, const std::function<void(const arma::vec &)> &set_joints_pos_fun, QWidget *parent): QDialog(parent)
{
  this->set_joints_pos_fun = set_joints_pos_fun;
  init(get_joints_pos_fun);
}


void SetJointsPosDialog::init(const std::function<arma::vec()> &get_joints_pos_fun)
{
  this->get_joints_pos_fun = get_joints_pos_fun;

  this->setWindowTitle("Set start pose");

  arma::vec q_start = this->get_joints_pos_fun();
  int N = q_start.size();

  // ========= pose layout ============

  joint_label.resize(N);
  for (int i=0; i<N; i++)
  {
    QString joint_name = (std::string("j")+std::to_string(i)).c_str();
    joint_label[i] = new QLabel(joint_name);
    joint_label[i]->setMinimumSize(75,40);
    joint_label[i]->setMaximumSize(75,40);
    joint_label[i]->setAlignment(Qt::AlignCenter);
    joint_label[i]->setFont(QFont("Ubuntu",14));
  }
  QHBoxLayout *joint_name_layout = new QHBoxLayout;
  for (int i=0; i<N; i++) joint_name_layout->addWidget(joint_label[i]);
  joint_name_layout->addStretch();

  jpos_le.resize(N);
  for (int i=0; i<N; i++)
  {
    jpos_le[i] = new QLineEdit;
    jpos_le[i]->setMinimumSize(75,40);
    jpos_le[i]->setMaximumSize(75,40);
    jpos_le[i]->setAlignment(Qt::AlignCenter);
    jpos_le[i]->setFont(QFont("Ubuntu",14));
    jpos_le[i]->setText("0");
  }
  QHBoxLayout *joint_val_layout = new QHBoxLayout;
  for (int i=0; i<N; i++) joint_val_layout->addWidget(jpos_le[i]);
  joint_val_layout->addStretch();

  // ========= buttons layout ============

  ok_btn = new QPushButton;
  ok_btn->setText("Ok");
  ok_btn->setFixedSize(80,40);
  ok_btn->setFont(QFont("Ubuntu",14));
  cancel_btn = new QPushButton;
  cancel_btn->setText("Cancel");
  cancel_btn->setFixedSize(80,40);
  cancel_btn->setFont(QFont("Ubuntu",14));
  QHBoxLayout *btns_layout = new QHBoxLayout;
  btns_layout->addWidget(ok_btn);
  btns_layout->addWidget(cancel_btn);
  btns_layout->addStretch();

  // ========= main layout ============

  QGridLayout *main_layout = new QGridLayout(this);
  main_layout->addLayout(joint_name_layout, 0, 0);
  main_layout->addLayout(joint_val_layout, 1, 0);
  main_layout->addItem(new QSpacerItem(0,25), 1, 0);
  main_layout->addLayout(btns_layout, 2, 0);

  // ========== connections  ============
  QObject::connect(ok_btn, &QPushButton::clicked, [this]()
  {
    arma::vec q_start(this->jpos_le.size());
    for (int i=0; i<this->jpos_le.size(); i++) q_start(i) = this->jpos_le[i]->text().toDouble();
    if (this->set_joints_pos_fun) this->set_joints_pos_fun(q_start);
    this->hide();
    emit this->jointsPosSetSignal();
  } );
  QObject::connect(cancel_btn, &QPushButton::clicked, [this](){ this->hide(); } );

  QObject::connect(this, &SetJointsPosDialog::setJointsPosSignal, [this](arma::vec j_pos_start)
  {
    for (int i=0; i<this->jpos_le.size(); i++) jpos_le[i]->setText(QString::number(j_pos_start(i),'f',2));
  });
}

SetJointsPosDialog::~SetJointsPosDialog()
{}

arma::vec SetJointsPosDialog::readJointsPos() const
{
  arma::vec pose(jpos_le.size());
  for (int i=0; i<pose.size(); i++) pose[i] = jpos_le[i]->text().toDouble();

  return pose;
}

void SetJointsPosDialog::launch()
{
  arma::vec q_start = this->get_joints_pos_fun();
  for (int i=0; i<jpos_le.size(); i++)
  {
    jpos_le[i]->setText(QString::number(q_start(i),'f',2));
  }

  this->show();
}

} // namespace gui_

} // namespace as64_
