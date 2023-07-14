#include <train_controller/utils/param_slider.h>

#include <iostream>

ParamSlider::ParamSlider(double min_val, double max_val, const std::string &param_name, QWidget *parent): QFrame(parent)
{
  is_slider_pressed = false;

  this->min_val = min_val;
  this->max_val = max_val;

  // set actual slider's range and in turn precision
  slider_max = 100000;
  slider_min = -100000;

  // =============  widgets  ============

  label = new QLabel(param_name.c_str());
  label->setStyleSheet("font: 75 14pt;");

  slider = new QSlider(Qt::Horizontal);
  slider->setRange(slider_min, slider_max);
  QObject::connect(slider, &QSlider::sliderPressed, this, [this]() { this->is_slider_pressed = true; });
  QObject::connect(slider, &QSlider::sliderReleased, this, [this]() { this->is_slider_pressed = false; });
  QObject::connect(slider, &QSlider::valueChanged, this, [this](int val)
  {
    slider->setSliderPosition(val);
    value->setText(QString::number(sliderPos2RealPos(val),'f',1));
    emit this->valueChanged();
  });

  value = new QLineEdit;
  value->setAlignment(Qt::AlignCenter);
  value->setMaximumWidth(65);
  value->setStyleSheet("font: 75 14pt;");
  QObject::connect( value, &QLineEdit::editingFinished, this, [this]()
  { slider->valueChanged( realPos2SliderPos(value->text().toDouble()) ); });

  QLabel *min_val_label = new QLabel("min:");
  min_val_label->setStyleSheet("font: 75 15pt;");

  min_val_le = new QLineEdit(QString::number(min_val));
  min_val_le->setAlignment(Qt::AlignCenter);
  min_val_le->setFixedWidth(65);
  min_val_le->setStyleSheet("font: 75 15pt;");
  QObject::connect( min_val_le, &QLineEdit::editingFinished, this, [this]()
  { this->min_val = min_val_le->text().toDouble(); });

  QLabel *max_val_label = new QLabel("max:");
  max_val_label->setStyleSheet("font: 75 15pt;");

  max_val_le = new QLineEdit(QString::number(max_val));
  max_val_le->setAlignment(Qt::AlignCenter);
  max_val_le->setFixedWidth(65);
  max_val_le->setStyleSheet("font: 75 15pt;");
  QObject::connect( max_val_le, &QLineEdit::editingFinished, this, [this]()
  { this->max_val = max_val_le->text().toDouble(); });

  // =============  layouts  ============
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  QHBoxLayout *slider_layout = new QHBoxLayout;
  QHBoxLayout *limits_layout = new QHBoxLayout;

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

  // ===========  initialize  ============
  emit slider->valueChanged(0);

}

ParamSlider::~ParamSlider()
{

}

double ParamSlider::getSliderPos()
{
  return sliderPos2RealPos(slider->sliderPosition());
}

void ParamSlider::updateSliderText()
{
  double r_val = getSliderPos();
  value->setText(QString::number(r_val,'f',1));
}

double ParamSlider::sliderPos2RealPos(int s_pos) const
{
  double s_min = slider->minimum();
  double s_max = slider->maximum();
  double r_pos = (s_pos-s_min)*(max_val-min_val)/(s_max-s_min) + min_val;
  return r_pos;
}

int ParamSlider::realPos2SliderPos(double r_pos) const
{
  double s_min = slider->minimum();
  double s_max = slider->maximum();
  double s_pos = static_cast<int>((r_pos-min_val)*(s_max-s_min)/(max_val-min_val) + s_min + 0.5);
  return s_pos;
}

void ParamSlider::setMinValue(double min_val)
{
  min_val_le->setText(QString::number(min_val));
  emit min_val_le->editingFinished();
}

void ParamSlider::setMaxValue(double max_val)
{
  max_val_le->setText(QString::number(max_val));
  emit max_val_le->editingFinished();
}
