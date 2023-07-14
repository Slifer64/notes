#ifndef TRAIN_CONTROLLER_GUI_PARAM_SLIDER_H
#define TRAIN_CONTROLLER_GUI_PARAM_SLIDER_H

#include <QLabel>
#include <QLineEdit>
#include <QSlider>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCheckBox>

#include <vector>
#include <cstring>
#include <armadillo>
#include <functional>
#include <thread>
#include <chrono>

class ParamSlider : public QFrame
{
Q_OBJECT

public:
  ParamSlider(double min_val, double max_val, const std::string &param_name="param", QWidget *parent = 0);
  ~ParamSlider();

  double getSliderPos();

  void setMinValue(double min_val);
  void setMaxValue(double max_val);

signals:
  void valueChanged();

private:

  bool is_slider_pressed;

  QLabel *label;
  QSlider *slider;
  QLineEdit *value;

  QLineEdit *min_val_le;
  QLineEdit *max_val_le;

  double min_val;
  double max_val;

  int slider_max;
  int slider_min;

  void updateGUI();

  void updateSliderText();

  double sliderPos2RealPos(int s_pos) const;
  int realPos2SliderPos(double r_pos) const;
};

#endif // TRAIN_CONTROLLER_GUI_PARAM_SLIDER_H
