#include <gmp_model/model.h>
#include <io_lib/print_utils.h>

#include <io_lib/file_io.h>
#include <math_lib/math_lib.h>

#include <ros/package.h>

#define Model_fun_ std::string("[Model::") + __func__ + "]: "

Model::Model()
{
  is_initialized_ = false;

  P0 = Pf = {0,0,0};
  Q0 = Qf = {1,0,0,0};

  Q_init = Q_target = {1,0,0,0};

  type = Model::Type::STD;

  initialize(2);

  // ros::NodeHandle nh("~");
  // if (!nh.getParam("model_path", default_model_path)) default_model_path = "";
  // default_model_path = ros::package::getPath("gmp_model") + "/" +  default_model_path;

  // if (!nh.getParam("model_path", default_model_path)) default_model_path = "";
  // default_model_path = ros::package::getPath("gmp_model") + "/" +  default_model_path;
  //
  // if (load_model)
  // {
  //   std::string model_filename;
  //   if (!nh.getParam("model_filename", model_filename)) PRINT_WARNING_MSG("Failed to load model...\n");
  //   else load(getDefaultPath() + model_filename);
  // }
}

Model::~Model()
{

}

void Model::initialize(unsigned N_kernels)
{
  double kernels_std_scaling = 1.5;

  gmp_p = gmp_::GMP(3, N_kernels, kernels_std_scaling);
  gmp_o = gmp_::GMPo(N_kernels, kernels_std_scaling);

  is_initialized_ = true;
}

void Model::train(const std::string &train_method, const arma::rowvec &Time, const arma::mat &Pos, const arma::mat &Quat)
{
  // ===========  Train GMP  ===============
  arma::wall_clock timer;
  arma::vec offline_train_mse;

  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized...\n");

  //setTargetPose(Pos.col(i_end), Quat.col(i_end));

  arma::rowvec x_data = Time / Time.back();

  x_train_data = x_data;

  // std::cout << gmp_p.numOfDoFs() << " x " << gmp_p.numOfKernels() << "\n";

  timer.tic();
  std::cout << "GMP_nDoF training...\n";
  std::cout << "Time: 1 x " <<  Time.size() << "\n";
  std::cout << "Pos: " <<  Pos.n_rows << " x " << Pos.n_cols << "\n";
  gmp_p.train(train_method, x_data, Pos, &offline_train_mse);
  std::cout << "offline_train_mse = \n" << offline_train_mse << "\n";
  std::cout << "Elapsed time: " << timer.toc() << " sec\n";

  timer.tic();
  std::cout << "GMPo training...\n";
  gmp_o.train(train_method, x_data, Quat, &offline_train_mse);
  std::cout << "offline_train_mse = \n" << offline_train_mse << "\n";
  std::cout << "Elapsed time: " << timer.toc() << " sec\n";

  Pf = gmp_p.getYd(1);
  Qf = gmp_o.getQd(1);
  P0 = gmp_p.getYd(0);
  Q0 = gmp_o.getQd(0);
  Tf = Time.back();

  is_initialized_ = true;
  is_trained_.set(true);
}

void Model::setTargetPose(const arma::vec &Pg, const arma::vec &Qg)
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");

  Q_target = Qg;
  if (arma::dot(Q_init,Q_target) < 0) Q_target = -Q_target;

  gmp_p.setGoal(Pg);
  gmp_o.setQg(Q_target);
}

void Model::setInitialPose(const arma::vec &P0, const arma::vec &Q0)
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");

  Q_init = Q0;
  if (arma::dot(Q_init,Q_target) < 0) Q_init = -Q_init;

  gmp_p.setY0(P0);
  gmp_o.setQ0(Q_init);
}

void Model::setPosStiffness(double Kp)
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");
  gmp_p.K = arma::vec().ones(3)*Kp;
}

void Model::setPosDamping(double Dp)
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");
  gmp_p.D = arma::vec().ones(3)*Dp;
}

void Model::setOrientStiffness(double Ko)
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");
  gmp_o.K = arma::vec().ones(3)*Ko;
}

void Model::setOrientDamping(double Do)
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");
  gmp_o.D = arma::vec().ones(3)*Do;
}


arma::vec Model::getRefPos(double x) const
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");
  return gmp_p.getYd(x);
}

arma::vec Model::getRefVel(double x, double x_dot) const
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");
  return gmp_p.getYdDot(x, x_dot);
}

arma::vec Model::getRefAccel(double x, double x_dot, double x_ddot) const
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");
  return gmp_p.getYdDDot(x, x_dot, x_ddot);
}

arma::vec Model::getRefOrient(double x) const
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");
  return gmp_o.getYd(x);
}

arma::vec Model::getRefOrientDot(double x, double x_dot) const
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");
  return gmp_o.getYdDot(x,x_dot);
}

arma::vec Model::getRefOrientDDot(double x, double x_dot, double x_ddot) const
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");
  return gmp_o.getYdDDot(x, x_dot, x_ddot);
}

arma::vec Model::getRefQuat(double x) const
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");
  return gmp_o.getQd(x);
}


arma::vec Model::getRefRotVel(double x, double x_dot) const
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");
  return gmp_o.getVd(x, x_dot);
}

arma::vec Model::getMotionDir(double x) const
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");
  double zero_tol = 1e-30;
  arma::vec dP_dir = gmp_p.getYdDot(x, 1);
  return dP_dir / ( arma::norm(dP_dir) + zero_tol );
}

arma::vec Model::calcAccel(const arma::vec &pos, const arma::vec &vel, double x, double x_dot, double x_ddot) const
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");
  return gmp_p.calcYddot(gmp_::Phase(x, x_dot, x_ddot), pos, vel);
}

arma::vec Model::calcOrientAccel(const arma::vec &q, const arma::vec &q_dot, double x, double x_dot, double x_ddot) const
{
  if (!is_initialized_) throw std::runtime_error(Model_fun_ + "The model is not initialized!");
  return gmp_o.calcYddot(gmp_::Phase(x, x_dot, x_ddot), q, q_dot);
}


void Model::initUpdate()
{
  gmp_p.deepCopy(&gmp_p_cp);
  gmp_p_up.reset(new gmp_::GMP_Update(&gmp_p_cp));
  gmp_p_up->recursiveUpdate(true);
  gmp_p_up->initSigmaWfromPosMsr(x_train_data);
  gmp_p_up->setMsrNoiseVar(1e-4);

  gmp_o.deepCopy(&gmp_o_cp);
  gmp_o_up.reset(new gmp_::GMPo_Update(&gmp_o_cp));
  gmp_o_up->recursiveUpdate(true);
  gmp_o_up->initSigmaWfromPosMsr(x_train_data);
  gmp_o_up->setMsrNoiseVar(1e-4);
}

void Model::updatePosition(double x, const arma::vec &P)
{
  gmp_p_up->updatePos(x, P);
}

void Model::updateqLogQuat(double x, const arma::vec &q)
{
  gmp_o_up->updatePos(x, q);
}

void Model::updatePositionParams()
{
  gmp_p = gmp_p_cp;
}

void Model::updateOrientationParams()
{
  gmp_o = gmp_o_cp;
}


bool Model::load(const std::string &path, std::string *err_msg)
{
  try
  {
    gmp_::FileIO fid(path, gmp_::FileIO::in);

    // initialize with some params...
    this->initialize(2);

    // int t;
    // fid.read("type", type);
    // if (t != getType()) throw std::runtime_error()

    gmp_::read(&gmp_p, fid, "pos_");
    gmp_::read(&gmp_o, fid, "orient_");

    fid.read("init positon", P0);
    fid.read("final positon", Pf);
    fid.read("init quat", Q0);
    fid.read("final quat", Qf);
    fid.read("train duration", Tf);
    fid.read("x_train_data", x_train_data);

    is_trained_.set(true);

    return true;
  }
  catch(std::exception &e){
    PRINT_ERROR_MSG(Model_fun_ + e.what() + "\n");
    if (err_msg) *err_msg = Model_fun_ + e.what();
    return false;
  }
}

bool Model::save(const std::string &path, std::string *err_msg)
{
  try
  {
    if (!is_trained_())
    {
      if (err_msg) *err_msg = "The model is not trained. Model save aborted...";
      return false;
    }

    gmp_::FileIO fid(path, gmp_::FileIO::out | gmp_::FileIO::trunc);
    fid.write("type", (int)type);
    gmp_::write(&gmp_p, fid, "pos_");
    gmp_::write(&gmp_o, fid, "orient_");
    fid.write("init positon", getTrainInitPos());
    fid.write("final positon", getTrainFinalPos());
    fid.write("init quat", getTrainInitQuat());
    fid.write("final quat", getTrainFinalQuat());
    fid.write("train duration", getTrainDuration());
    fid.write("x_train_data", x_train_data);
    
    return true;
  }
  catch(std::exception &e){
    PRINT_ERROR_MSG(Model_fun_ + e.what() + "\n");
    if (err_msg) *err_msg = Model_fun_ + e.what();
    return false;
  }
}

arma::mat Model::getPositionPath(const arma::vec &P0, const arma::vec &Pg, const arma::vec &Q0, const arma::vec &Qg, double duration)
{
  if (!isTrained()) throw std::runtime_error(Model_fun_ + "The model is not trained!");

  double tau = duration;
  double x = 0;
  double x_dot = 1/tau;
  double dt = duration/300; //0.005;

  arma::rowvec Time = arma::linspace<arma::rowvec>(0, tau, std::round(tau/dt));
  int n_data = Time.size();
  arma::mat P_data(3, n_data);

  this->setInitialPose(P0, Q0);
  this->setTargetPose(Pg, Qg);

  for (int j=0; j<n_data; j++)
  {
    x = Time(j)/tau;
    P_data.col(j) = this->getRefPos(x);
  }

  return P_data;
}

arma::mat Model::getQuatPath(const arma::vec &P0, const arma::vec &Pg, const arma::vec &Q0, const arma::vec &Qg, double duration)
{
  if (!isTrained()) throw std::runtime_error(Model_fun_ + "The model is not trained!");

  double tau = duration;
  double x = 0;
  double x_dot = 1/tau;
  double dt = duration/300; //0.005;

  arma::rowvec Time = arma::linspace<arma::rowvec>(0, tau, std::round(tau/dt));
  int n_data = Time.size();
  arma::mat Q_data(4, n_data);

  this->setInitialPose(P0, Q0);
  this->setTargetPose(Pf, Qg);

  for (int j=0; j<n_data; j++)
  {
    x = Time(j)/tau;
    Q_data.col(j) = getRefQuat(x); // this->orientToQuat(this->getRefOrient(x), Q0);
  }

  return Q_data;
}

// ==============  static functions  =============

arma::vec Model::quatToOrient(const arma::vec &Q, const arma::vec &Q0)
{
  return math_::quatLog(gmp_::GMPo::getQ1(Q, Q0));
}

arma::vec Model::orientToQuat(const arma::vec &q, const arma::vec &Q0)
{
  return gmp_::GMPo::q2quat(q, Q0);
}

arma::vec Model::orientVelToRotVel(const arma::vec &q_dot, const arma::vec &Q, const arma::vec &Q0)
{
  return math_::qLogDot_to_rotVel(q_dot, gmp_::GMPo::getQ1(Q, Q0));
}
