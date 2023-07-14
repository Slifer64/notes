#ifndef GUI_LIB_PHASE_CTRL_SLIDER_H
#define GUI_LIB_PHASE_CTRL_SLIDER_H

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QSlider>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCloseEvent>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>

#include <vector>
#include <cstring>
#include <armadillo>
#include <functional>
#include <thread>
#include <chrono>

namespace as64_
{

namespace gui_
{


class PhaseCtrlSlider : public QFrame
{
Q_OBJECT

public:
  PhaseCtrlSlider(double min_val, double max_val, QWidget *parent = 0);
  ~PhaseCtrlSlider();

  double getSliderPos();

  void setMinValue(double min_val);
  void setMaxValue(double max_val);
  void setResetRate(double reset_rate);
  void autoReset(bool set);

signals:
  void updateSliderPosition(int pos);

private:

  void launchResetSliderThread();

  bool is_slider_pressed;
  bool reset_slider;
  double reset_rate;

  QCheckBox *reset_chkbox;
  QLineEdit *reset_rate_le;
  QLabel *reset_rate_label;
  QPushButton *reset_btn;


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

  void chkBoxChanged();

  void updateSliderText();

  double sliderPos2RealPos(int s_pos) const;
  int realPos2SliderPos(double r_pos) const;
};

} // namespace gui_

} // namespace as64_

#endif // GUI_LIB_PHASE_CTRL_SLIDER_H
