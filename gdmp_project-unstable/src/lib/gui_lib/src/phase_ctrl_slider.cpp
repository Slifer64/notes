#include <gui_lib/phase_ctrl_slider.h>

#include <iostream>

namespace as64_
{

namespace gui_
{


PhaseCtrlSlider::PhaseCtrlSlider(double min_val, double max_val, QWidget *parent): QFrame(parent)
{
  is_slider_pressed = false;
  reset_slider = false;
  reset_rate = 0;

  this->min_val = min_val;
  this->max_val = max_val;

  slider_max = 1000;
  slider_min = -1000;

  // =============  widgets  ============

  label = new QLabel("fv");
  label->setStyleSheet("font: 75 14pt;");

  slider = new QSlider(Qt::Horizontal);
  slider->setRange(slider_min, slider_max);
  QObject::connect(slider, &QSlider::sliderPressed, this, [this]() { this->is_slider_pressed = true; });
  QObject::connect(slider, &QSlider::sliderReleased, this, [this]() { this->is_slider_pressed = false; });
  QObject::connect(slider, &QSlider::valueChanged, this, [this]() { this->updateSliderText(); });

  value = new QLineEdit;
  value->setAlignment(Qt::AlignCenter);
  value->setMaximumWidth(65);
  value->setStyleSheet("font: 75 14pt;");

  reset_chkbox = new QCheckBox("Auto-Reset");
  reset_chkbox->setChecked(true);
  QObject::connect(reset_chkbox, &QCheckBox::stateChanged, this, [this](){ this->chkBoxChanged(); });

  reset_rate_label = new QLabel("Reset rate:");
  reset_rate_label->setStyleSheet("font: 75 15pt;");

  reset_rate_le = new QLineEdit("0.05");
  reset_rate_le->setAlignment(Qt::AlignCenter);
  reset_rate_le->setFixedWidth(65);
  reset_rate_le->setStyleSheet("font: 75 15pt;");
  QObject::connect( reset_rate_le, &QLineEdit::editingFinished, this, [this]()
  {
    reset_rate = reset_rate_le->text().toDouble();
    if (reset_rate < 0)
    {
      reset_rate = 0;
      reset_rate_le->setText("0");
    }
  });

  reset_btn = new QPushButton("reset");
  reset_btn->setStyleSheet("font: 75 15pt;");
  QObject::connect( reset_btn, &QPushButton::pressed, this, [this]()
  { slider->setValue(realPos2SliderPos(0)); });


  QLabel *min_val_label = new QLabel("min:");
  min_val_label->setStyleSheet("font: 75 15pt;");

  min_val_le = new QLineEdit(QString::number(min_val));
  min_val_le->setAlignment(Qt::AlignCenter);
  min_val_le->setFixedWidth(65);
  min_val_le->setStyleSheet("font: 75 15pt;");
  QObject::connect( min_val_le, &QLineEdit::editingFinished, this, [this]()
  {
    this->min_val = min_val_le->text().toDouble();
    slider->setValue(realPos2SliderPos(0));
  });

  QLabel *max_val_label = new QLabel("max:");
  max_val_label->setStyleSheet("font: 75 15pt;");

  max_val_le = new QLineEdit(QString::number(max_val));
  max_val_le->setAlignment(Qt::AlignCenter);
  max_val_le->setFixedWidth(65);
  max_val_le->setStyleSheet("font: 75 15pt;");
  QObject::connect( max_val_le, &QLineEdit::editingFinished, this, [this]()
  {
    this->max_val = max_val_le->text().toDouble();
    slider->setValue(realPos2SliderPos(0));
  });

  // =============  layouts  ============
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  QHBoxLayout *slider_layout = new QHBoxLayout;
  QHBoxLayout *reset_layout = new QHBoxLayout;
  QHBoxLayout *limits_layout = new QHBoxLayout;

  reset_layout->addWidget(reset_chkbox);
  reset_layout->addSpacing(15);
  reset_layout->addWidget(reset_rate_label);
  reset_layout->addWidget(reset_rate_le);
  reset_layout->addSpacing(20);
  reset_layout->addWidget(reset_btn);
  reset_layout->addStretch();

  limits_layout->addWidget(min_val_label);
  limits_layout->addWidget(min_val_le);
  limits_layout->addSpacing(15);
  limits_layout->addWidget(max_val_label);
  limits_layout->addWidget(max_val_le);
  limits_layout->addStretch(0);

  slider_layout->addWidget(label);
  slider_layout->addWidget(slider);
  slider_layout->addWidget(value);

  main_layout->addLayout(limits_layout);
  main_layout->addLayout(slider_layout);
  main_layout->addLayout(reset_layout);


  // ===========  initialize  ============
  emit reset_chkbox->stateChanged(Qt::Unchecked);
  emit slider->valueChanged(0);

}

PhaseCtrlSlider::~PhaseCtrlSlider()
{

}

void PhaseCtrlSlider::launchResetSliderThread()
{
  std::thread([this]()
  {
    while (reset_slider)
    {
      if (!is_slider_pressed)
      {
        double pos = getSliderPos();
        int sign_ = 2*(pos>=0)-1;
        pos = pos - sign_*reset_rate;
        if (pos*sign_<0) pos = 0;
        slider->setValue(realPos2SliderPos(pos));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  }).detach();
}

double PhaseCtrlSlider::getSliderPos()
{
  return sliderPos2RealPos(slider->sliderPosition());
}

void PhaseCtrlSlider::chkBoxChanged()
{
  bool set = reset_chkbox->isChecked();

  reset_rate_le->setEnabled(set);

  reset_slider = set;

  if (set)
  {
      reset_rate = reset_rate_le->text().toDouble();
      launchResetSliderThread();
  }
}

void PhaseCtrlSlider::updateSliderText()
{
  double r_val = getSliderPos();
  value->setText(QString::number(r_val,'f',1));
}

double PhaseCtrlSlider::sliderPos2RealPos(int s_pos) const
{
  double s_min = slider->minimum();
  double s_max = slider->maximum();
  double r_pos = (s_pos-s_min)*(max_val-min_val)/(s_max-s_min) + min_val;
  return r_pos;
}

int PhaseCtrlSlider::realPos2SliderPos(double r_pos) const
{
  double s_min = slider->minimum();
  double s_max = slider->maximum();
  double s_pos = static_cast<int>((r_pos-min_val)*(s_max-s_min)/(max_val-min_val) + s_min + 0.5);
  return s_pos;
}

void PhaseCtrlSlider::setMinValue(double min_val)
{
  min_val_le->setText(QString::number(min_val));
  emit min_val_le->editingFinished();
}

void PhaseCtrlSlider::setMaxValue(double max_val)
{
  max_val_le->setText(QString::number(max_val));
  emit max_val_le->editingFinished();
}

void PhaseCtrlSlider::setResetRate(double reset_rate)
{
  reset_rate_le->setText(QString::number(reset_rate));
  emit reset_rate_le->editingFinished();
}

void PhaseCtrlSlider::autoReset(bool set)
{
  reset_chkbox->setChecked(set);
  emit reset_chkbox->stateChanged(0);
}


} // namespace gui_

} // namespace as64_