#include <gui_lib/set_pose_dialog.h>

#include <iomanip>

namespace as64_
{

namespace gui_
{

SetPoseDialog::SetPoseDialog(QWidget *parent): QDialog(parent)
{
  run = false;

   this->setWindowTitle("Set pose");

  QLabel *pos_label = new QLabel("Position");
  pos_label->setStyleSheet("background-color: rgb(245,245,245); color: rgb(0,0,0); font: 75 14pt \"FreeSans\";");
  QLabel *x_label = new QLabel("x");
  QLabel *y_label = new QLabel("y");
  QLabel *z_label = new QLabel("z");
  QLabel *m_label = new QLabel("m");

  x_le = createLineEdit();
  y_le = createLineEdit();
  z_le = createLineEdit();

  QLabel *orient_label = new QLabel("Orientation\n(Quaternion)");
  orient_label->setStyleSheet("background-color: rgb(245,245,245); color: rgb(0,0,0); font: 75 14pt \"FreeSans\";");
  QLabel *scalar_label = new QLabel("scalar");
  QLabel *vector_label = new QLabel("vector");

  qw_le = createLineEdit();
  qx_le = createLineEdit();
  qy_le = createLineEdit();
  qz_le = createLineEdit();


  QGridLayout *pose_layout = new QGridLayout;
  // pose_layout->setSizeConstraint(QLayout::SetFixedSize);
  pose_layout->addWidget(x_label,0,1, Qt::AlignCenter);
  pose_layout->addWidget(y_label,0,2, Qt::AlignCenter);
  pose_layout->addWidget(z_label,0,3, Qt::AlignCenter);
  pose_layout->addWidget(pos_label,1,0, Qt::AlignCenter);
  pose_layout->addWidget(x_le,1,1);
  pose_layout->addWidget(y_le,1,2);
  pose_layout->addWidget(z_le,1,3);
  pose_layout->addWidget(m_label,1,4);
  pose_layout->addItem(new QSpacerItem(0,20),2,0);
  pose_layout->addWidget(orient_label,3,0,2,1, Qt::AlignCenter);
  pose_layout->addWidget(scalar_label,3,1, Qt::AlignCenter);
  pose_layout->addWidget(vector_label,3,2,1,3, Qt::AlignCenter);
  pose_layout->addWidget(qw_le,4,1);
  pose_layout->addWidget(qx_le,4,2);
  pose_layout->addWidget(qy_le,4,3);
  pose_layout->addWidget(qz_le,4,4);

  // --------------------------------------------------
  ok_btn = new QPushButton("ok");
  ok_btn->setStyleSheet("font: 75 14pt;");
  QObject::connect( ok_btn, &QPushButton::pressed, this, [this]()
  {
    this->stop();

    arma::vec new_pose(7);
    new_pose(0) = x_le->text().toDouble();
    new_pose(1) = y_le->text().toDouble();
    new_pose(2) = z_le->text().toDouble();

    new_pose(3) = qw_le->text().toDouble();
    new_pose(4) = qx_le->text().toDouble();
    new_pose(5) = qy_le->text().toDouble();
    new_pose(6) = qz_le->text().toDouble();

    setPose(new_pose);

    emit this->poseChanged(true);
  });

  cancel_btn = new QPushButton("cancel");
  cancel_btn->setStyleSheet("font: 75 14pt;");
  QObject::connect( cancel_btn, &QPushButton::pressed, this, [this]() { this->stop(); });

  QHBoxLayout *btns_layout = new QHBoxLayout;
  btns_layout->addStretch(0);
  btns_layout->addWidget(ok_btn);
  btns_layout->addWidget(cancel_btn);

  // --------------------------------------------------

  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->addLayout(pose_layout);
  main_layout->addLayout(btns_layout, Qt::AlignRight);

  pose = {0,0,0, 1,0,0,0};
}

SetPoseDialog::~SetPoseDialog()
{
  stop();
}

arma::vec SetPoseDialog::getPose() const
{
  // arma::vec pose(7);
  //
  // pose(0) = x_le->text().toDouble();
  // pose(1) = y_le->text().toDouble();
  // pose(2) = z_le->text().toDouble();
  //
  // pose(3) = qw_le->text().toDouble();
  // pose(4) = qx_le->text().toDouble();
  // pose(5) = qy_le->text().toDouble();
  // pose(6) = qz_le->text().toDouble();
  return pose;
}

void SetPoseDialog::setPose(const arma::vec &pose)
{
  auto num2str = [](double num, int prec=3)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(prec) << num;
    return oss.str().c_str();
  };

  this->pose = pose;
  this->pose.subvec(3,6) /= arma::norm(this->pose.subvec(3,6)); // normalize quaternion

  x_le->setText( num2str(pose(0)) );
  y_le->setText( num2str(pose(1)) );
  z_le->setText( num2str(pose(2)) );
  qw_le->setText( num2str(pose(3)) );
  qx_le->setText( num2str(pose(4)) );
  qy_le->setText( num2str(pose(5)) );
  qz_le->setText( num2str(pose(6)) );
}

MyLineEdit *SetPoseDialog::createLineEdit()
{
  MyLineEdit *le = new MyLineEdit;
  le->setSizeHint(60,30);
  le->setStyleSheet("font: 75 14pt;");
  le->setAlignment(Qt::AlignCenter);

  return le;
}

void SetPoseDialog::launch()
{
  if (!run)
  {
    run = true;
    this->show();
  }
}

void SetPoseDialog::stop()
{
  if (run)
  {
      run = false;
      this->hide();
  }
}

void SetPoseDialog::closeEvent(QCloseEvent *event)
{
  stop();
}

} // namespace gui_

} // namespace as64_