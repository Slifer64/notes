#ifndef GUI_LIB_SET_JOINTS_POSITION_DIALOG_H
#define GUI_LIB_SET_JOINTS_POSITION_DIALOG_H

#include <QLineEdit>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QComboBox>
#include <QCheckBox>
#include <QScrollArea>
#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QDialog>
#include <QSpacerItem>
#include <QPalette>

#include <armadillo>

namespace as64_
{

namespace gui_
{

/** Opens a Qt dialog for displaying and setting joints positions;
 */
class SetJointsPosDialog: public QDialog
{
  Q_OBJECT

public:
  /** Constructor
   * @param get_joints_pos_fun pointer to function that returns the joint position values to be displayed in this gui.
   * @param parent the parent widget (optional, default=0).
   */
  SetJointsPosDialog(const std::function<arma::vec()> &get_joints_pos_fun, QWidget *parent=0);

  /** Constructor 2
   * @param get_joints_pos_fun pointer to function that returns the joint position values to be displayed in this gui.
   * @param set_joints_pos_fun pointer to function that sets the joints positions (can be used for updating the joint \n
   *                           position values of another variable, when this gui's values change.)
   * @param parent the parent widget (optional, default=0).
   */
  SetJointsPosDialog(const std::function<arma::vec()> &get_joints_pos_fun, const std::function<void(const arma::vec &)> &set_joints_pos_fun, QWidget *parent=0);

  /** Destructor
   */
  ~SetJointsPosDialog();

  /** Returns the joint position values displayed on the gui.
   * @return the joint position values of the gui.
   */
  arma::vec readJointsPos() const;


signals:
  /** Qt signal, emitted whenever 'ok' is pressed on the gui, for submitting the current values.
   */
  void jointsPosSetSignal();

  /** Qt signal, that sets the joint values to be displayed on the gui.
   * @param j_pos joint position values to be displayed on the gui.
   */
  void setJointsPosSignal(arma::vec j_pos);

public slots:

  /** Makes the gui visible.
   */
  void launch();

private:

  void init(const std::function<arma::vec()> &get_joints_pos_fun);

  std::vector<QLineEdit *> jpos_le;
  std::vector<QLabel *> joint_label;
  QPushButton *ok_btn;
  QPushButton *cancel_btn;

  std::function<arma::vec()> get_joints_pos_fun;
  std::function<void(const arma::vec &)> set_joints_pos_fun;
};

} // namespace gui_

} // namespace as64_

#endif // GUI_LIB_SET_JOINTS_POSITION_DIALOG_H
