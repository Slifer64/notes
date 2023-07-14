#include <im_traj_demo_controller/controller.h>

#include <iostream>
#include <io_lib/xml_parser.h>
#include <io_lib/file_io.h>
#include <math_lib/math_lib.h>
#include <gmp_lib/io/gmp_io.h>
// #include <matlab_lib/mat_file_writer.h>

#include <ros/package.h>

#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

using namespace as64_;


#define ImTrajDemoController_fun_ std::string("[ImTrajDemoController::") + __func__ + "]: "

ImTrajDemoController::ImTrajDemoController(MainController *main_ctrl, const std::string &ctrl_name)
: Controller(main_ctrl, ctrl_name), async_spinner(2)
{
  // Gb_::train_ctrl = this;

  async_spinner.start();

  this->main_ctrl = main_ctrl;
  this->robot = main_ctrl->robot.get();

  ros::NodeHandle nh("~");

  default_data_path = ros::package::getPath("im_traj_demo_controller") + "/";

  // load params
  loadParams();

  // setup rviz
  rviz_pub.reset( new rviz_::RvizMarkerPublisher("/" + getNodeNameID() + "_im_traj_demo_ctrl_marker_topic", base_link) );

  // subscribe to rgb image topic
  img_reader.reset(new ros_lib_::ImageReader(params.image_topic, params.depth_image_topic));

  // subscribe to robot_cam tf topic
  robot_cam_tf.reset(new ros_lib_::TransformReader(params.robot_cam_tf_topic));

  // create client to move the camera
  if (!params.move2jpos_topic.empty())
    move2jpos_client.reset(new MoveToJointsPosActionClient(params.move2jpos_topic));

  // read camera info
  ros_lib_::CameraParams cam_params;
  if (!cam_params.readFromTopic(params.cam_info_topic, 10000))
    throw std::runtime_error(ImTrajDemoController_fun_ + "Timeout on waiting to read camera info from topic'");
  Cam_Proj_mat = arma::mat(cam_params.getCamProjMat().data(), 3, 3);
  //  arma::matauto cam_info = cam_params.getInfo();
  //  arma::mat(&cam_info.K[0], 3, 3).t();
  // arma::mat P = arma::mat(&cam_info.P[0], 4, 3).t();
  // std::cerr << "K = \n" << Cam_Proj_mat << "\n";
  // std::cerr << "P = \n" << P << "\n";
  // exit(-1);
}

QPushButton *ImTrajDemoController::createGui(MainWindow *parent)
{
  QPushButton *btn = new QPushButton(this->ctrl_name.c_str());
  ImTragDemoGui *gui = new ImTragDemoGui(this, parent);
  QObject::connect( btn, &QPushButton::pressed, parent, [gui](){ gui->launch(); });
  return btn;
}

ImTrajDemoController::~ImTrajDemoController()
{}

void ImTrajDemoController::loadParams(std::string path)
{
  if (path.empty()) path = default_data_path + "/config/params.yaml";

  ros::NodeHandle nh("~");
  if (!nh.getParam("base_link", base_link)) throw std::runtime_error(ImTrajDemoController_fun_ + "Failed to load param \"base_link\"...");
  if (!nh.getParam("image_topic", params.image_topic)) throw std::runtime_error(ImTrajDemoController_fun_ + "Failed to load param 'image_topic'");
  if (!nh.getParam("depth_image_topic", params.depth_image_topic))
  {
    PRINT_WARNING_MSG("params.depth_image_topic set to ''\n");
    params.depth_image_topic = "";
  }
  if (!nh.getParam("cam_info_topic", params.cam_info_topic)) throw std::runtime_error(ImTrajDemoController_fun_ + "Failed to load param 'cam_info_topic'");
  if (!nh.getParam("robot_cam_tf_topic", params.robot_cam_tf_topic)) throw std::runtime_error(ImTrajDemoController_fun_ + "Failed to load param 'robot_cam_tf_topic'");
  if (!nh.getParam("move2jpos_topic", params.move2jpos_topic)) throw std::runtime_error(ImTrajDemoController_fun_ + "Failed to load param 'move2jpos_topic'");
  
  // io_::XmlParser parser(default_data_path + "/config/params.yaml");
  YAML::Node node = YAML::LoadFile(path);
  if (!YAML::getParam(node, "N_kernels", params.N_kernels)) throw std::runtime_error(ImTrajDemoController_fun_ + "Failed to load param 'N_kernels'");
  if (!YAML::getParam(node, "im_capt_joint_pos", params.im_capt_joint_pos)) throw std::runtime_error(ImTrajDemoController_fun_ + "Failed to load param 'im_capt_joint_pos'");

  // std::cerr << "JOints:\n";
  // for (auto jpos : params.im_capt_joint_pos)
  // {
  //   std::cerr << arma::rowvec(jpos) << "\n";
  // }

  // YAML::Node cam_node;
  // if (!YAML::getParam(node, "camera_intrinsics", cam_node)) throw std::runtime_error(ImTrajDemoController_fun_ + "Failed to load param 'camera_intrinsics'");
  // if (!YAML::getParam(cam_node, "fx", camera_intrinsics.fx)) throw std::runtime_error(ImTrajDemoController_fun_ + "Failed to load param 'camera_intrinsics.fx'");
  // if (!YAML::getParam(cam_node, "fy", camera_intrinsics.fy)) throw std::runtime_error(ImTrajDemoController_fun_ + "Failed to load param 'camera_intrinsics.fy'");
  // if (!YAML::getParam(cam_node, "cx", camera_intrinsics.cx)) throw std::runtime_error(ImTrajDemoController_fun_ + "Failed to load param 'camera_intrinsics.cx'");
  // if (!YAML::getParam(cam_node, "cy", camera_intrinsics.cy)) throw std::runtime_error(ImTrajDemoController_fun_ + "Failed to load param 'camera_intrinsics.cy'");
}

// ===============================================================

void ImTrajDemoController::captureMultiImages()
{
  for (auto jpos : params.im_capt_joint_pos)
  {
    PRINT_INFO_MSG("Sending goal...\n");
    move2jpos_client->sendGoal(jpos);

    PRINT_INFO_MSG("Waiting...\n");
    while (true)
    {
      auto feedback = move2jpos_client->feedback();
      if (feedback.finished) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    auto result_ = move2jpos_client->result();

    PRINT_INFO_MSG("Finished...\n");

    if (!result_.success)
    {
      emit gui->abortMultiImgCaptSignal(result_.msg.c_str());
      return;
    }

    PRINT_INFO_MSG("Capture Image...\n");
    captureImage();
  }
}

void ImTrajDemoController::captureImage()
{
  if (!img_reader->readNewFrame(500))
    throw std::runtime_error(ImTrajDemoController_fun_ + "Timeout on wait to read image...");
  images.push_back(img_reader->getRGB());
  if (img_reader->hasDepth()) depth_images.push_back(img_reader->getDepth());
  PRINT_INFO_MSG("Captured image # " + std::to_string(images.size()) + "\n");

  if (!robot_cam_tf->isUpdated()) PRINT_WARNING_MSG(ImTrajDemoController_fun_ + "robot_cam_tf has not been updated...\n");
  robot_cam_tf_data.push_back(arma::join_vert(robot_cam_tf->pos(), robot_cam_tf->quat()));

  gui->setImageCount(images.size());

  is_image_captured = true;

  PRINT_INFO_MSG(ImTrajDemoController_fun_ + "Captured image!\n");
}

void ImTrajDemoController::clearImages()
{
  is_image_captured = false;
  images.clear();
  depth_images.clear();
  robot_cam_tf_data.clear();
  gui->setImageCount(0);
}

void ImTrajDemoController::startDemoRec()
{
  pathTeaching();
  exec_stop_sem.notify();
}

ExecResultMsg ImTrajDemoController::stopDemoRec()
{
  exec_on.set(false);
  if ( exec_stop_sem.wait_for(1500) ) return ExecResultMsg(ExecResultMsg::INFO, "Data recording stopped!");
  else return ExecResultMsg(ExecResultMsg::WARNING, "Time-out reached on waiting for stop data recording...");
}

void ImTrajDemoController::pathTeaching()
{
  // if (main_ctrl->robot->getMode() != rw_::Mode::FREEDRIVE) main_ctrl->setMode(rw_::Mode::FREEDRIVE);

  exec_on.set(true);

  robot->update();

  double Ts = robot->getCtrlCycle();

  double t = 0;
  arma::vec P = robot->getTaskPosition();
  arma::vec Q = robot->getTaskOrientation();
  arma::vec Q_prev = Q;

  log.clear();

  while (exec_on())
  {
    if (!robot->isOk())
    {
      // showErrorMsg("The robot is not ok...\n Aborting velocity teaching...");
      emit gui->stopTrainingSignal(ExecResultMsg(ExecResultMsg::ERROR, "The robot is not ok...\n Aborting path teaching..."));
      break;
    }

    // double sin_theta = arma::norm(Q.subvec(1, 3));
    // double cos_theta = Q(0);
    // double theta = 2*std::atan2(sin_theta, cos_theta);
    log.add(t, P, Q);
    // ====================================

    robot->update();
    t = t + Ts;
    P = robot->getTaskPosition();
    Q_prev = Q;
    Q = robot->getTaskOrientation();

    if (arma::dot(Q, Q_prev)<0) Q = -Q;
  }

  log.trim();

  mp.reset(new gmp_::GMP(3, params.N_kernels, 1.5));
  mp_o.reset(new gmp_::GMPo(params.N_kernels, 1.5));
  arma::rowvec s_data = log.Time / log.Time.back();
  arma::vec train_err;
  mp->train("LS", s_data, log.P_data, &train_err);
  std::cerr << "pos_train_err = " << train_err.t() << "\n";
  mp_o->train("LS", s_data, log.Q_data, &train_err);
  std::cerr << "orient_train_err = " << train_err.t() << "\n";
  is_demo_captured = true;
}

std::string zfill(const std::string &s, int fill, char fill_char='0')
{
  std::string s_fill(s);
  fill = fill - s.length();
  if (fill > 0) s_fill = std::string(fill, fill_char) + s_fill;
  return s_fill;
}

ExecResultMsg ImTrajDemoController::saveDemo()
{
  if (!is_demo_captured) return ExecResultMsg(ExecResultMsg::WARNING, "There is no recorded trajectory!\nSave aborted...");
  if (!is_image_captured) return ExecResultMsg(ExecResultMsg::WARNING, "There are no captured images!\nSave aborted...");

  // sample trajectory of 'n_points'
  int n_points = 200;
  arma::mat Pos0(3, n_points);
  arma::mat Quat0(4, n_points);
  arma::rowvec s_data = arma::linspace<arma::rowvec>(0, 1, n_points);
  for (int j=0; j < n_points; j++)
  {
    Pos0.col(j) = mp->getYd(s_data(j));
    Quat0.col(j) = mp_o->getQd(s_data(j));
  }

  // iterate over all captured images
  std::string log_path = getDefaultPath() + "/log/";
  int n_img = images.size();
  int sample_count = gui->getSaveCounter();
  for (int i=0; i<n_img; i++)
  {
    PRINT_INFO_MSG("Processing demo " + std::to_string(i+1) + "/" + std::to_string(n_img) + "...\n");

    // create sample folder
    std::string folder_name = log_path + zfill(std::to_string(sample_count), 5) + "/";
    if (fs::exists(folder_name)) fs::remove_all(folder_name);

    // std::cerr << "Creating directory: " << folder_name << "\n";
    fs::create_directory(folder_name);

    // save image
    cv::Mat bgr_img;
    cv::cvtColor(images[i], bgr_img, cv::COLOR_RGB2BGR);
    cv::imwrite(folder_name + "/rgb.png", bgr_img);
    if (!depth_images.empty()) cv::imwrite(folder_name + "/depth.png", depth_images[i]);

    // tranform trajectory from robot to camera frame
    arma::vec p_rc = robot_cam_tf_data[i].subvec(0, 2);
    arma::vec Q_rc = robot_cam_tf_data[i].subvec(3, 6);
    arma::mat R_rc = math_::quat2rotm(Q_rc);

    arma::mat Pos = Cam_Proj_mat * R_rc.t() * (Pos0 - arma::repmat(p_rc, 1, n_points));
    arma::mat xy_data = Pos.rows(0, 1) / arma::repmat(Pos.row(2), 2, 1);

    // # Extract the x and y pixel coordinates
    // x_pixel = int(u_distorted)
    // y_pixel = int(v_distorted)

    // No need to transform orientation to camera frame
    // arma::mat Qm_cr = math_::quat2qmat(math_::quatInv(Q_rc));
    // arma::mat Quat = Qm_cr * Quat0;

    arma::rowvec cos_theta2 = Quat0.row(0);
    arma::rowvec sin_theta2 = arma::sign(Quat0.row(3)) % arma::sqrt(arma::sum(arma::square(Quat0.rows(1, 3)), 0)); // sign_z * norm_k
    arma::rowvec theta_data = 2*arma::atan2(sin_theta2, cos_theta2);

    arma::mat y_data = arma::join_vert(xy_data, theta_data);

    gmp_::GMP mp2(3, params.N_kernels);
    arma::vec train_err;
    mp2.train("LS", s_data, y_data, &train_err);
    if (arma::norm(train_err) > 1e-3)
    {
      PRINT_WARNING_MSG("Training error may be large...!\n");
    }

    if (!mp2.W.save(folder_name + "/mp_weights.txt", arma::raw_ascii))
    {
      PRINT_WARNING_MSG("Error saving mp_weights...\n");
    }

    // save robot-cam tf
    if (!robot_cam_tf_data[i].save(folder_name + "/robot_cam_tf.txt", arma::raw_ascii))
    {
      PRINT_WARNING_MSG("Error saving robot_cam_tf...\n");
    }

    // save camera projection matrix
    if (!Cam_Proj_mat.save(folder_name + "/cam_proj_mat.txt", arma::raw_ascii))
    {
      PRINT_WARNING_MSG("Error saving cam_proj_mat...\n");
    }

    // save also the initial demo data and the trained model
    try
    {
      gmp_::FileIO fid(log_path + "/proj_demo.bin", io_::FileIO::out | io_::FileIO::trunc);
      
    }
    catch (std::exception &e)
    { PRINT_ERROR_MSG(ImTrajDemoController_fun_ + e.what()); }

    // save the projected data
    try
    {
      gmp_::FileIO fid(log_path + "/demo.bin", io_::FileIO::out | io_::FileIO::trunc);
      fid.write("Timed", log.Time);
      fid.write("Pd_data", log.P_data);
      fid.write("Qd_data", log.Q_data);
      gmp_::write(mp.get(), fid, "mp_pos");
      gmp_::write(mp_o.get(), fid, "mp_orient");

      fid.write("p_rc", p_rc);
      fid.write("Q_rc", Q_rc);
      fid.write("cam_proj_mat", Cam_Proj_mat);

      fid.write("xy_data", xy_data);
      fid.write("theta_data", theta_data);
      gmp_::write(&mp2, fid, "mp2");

    }
    catch (std::exception &e)
    { PRINT_ERROR_MSG(ImTrajDemoController_fun_ + e.what()); }
    
    sample_count++;
    gui->setSaveCounter(sample_count);
  }
  clearImages();
  PRINT_INFO_MSG("Finished demo processing!\n");
  return ExecResultMsg(ExecResultMsg::INFO, "The data have been saved successfully!");
}

void ImTrajDemoController::viewLearnedPath(bool view)
{
  if (view)
  {
    int n_data = 100;
    arma::rowvec s_data = arma::linspace<arma::rowvec>(0, 1, n_data);
    arma::mat y_data(3, n_data);
    for (int j=0; j < n_data; j++) y_data.col(j) = mp->getYd(s_data(j));

    arma::mat xy_pos = y_data.rows(0, 1);
    arma::rowvec z_pos = arma::rowvec().ones(n_data) * 0.1;
    arma::mat Pos = arma::join_vert(xy_pos, z_pos);

    arma::mat Quat = arma::mat().zeros(4, n_data);
    // arma::rowvec z_theta = y_data.row(2);
    // for (int j=0; j<n_data; j++) Quat.col(j) = math_::axang2quat(arma::vec{0, 0, 1, z_theta(j)});
    for (int j=0; j<n_data; j++) Quat.col(j) = mp_o->getQd(s_data(j));
    
    rviz_pub->publishOrientedPath(Pos, Quat, 6, rviz_::Color::MAGENTA, 0.02, 1.2, "learned_path");
    rviz_pub->drawnow();
  }
  else
  {
    rviz_pub->deleteMarkers("learned_path");
    rviz_pub->drawnow();
  }
}
