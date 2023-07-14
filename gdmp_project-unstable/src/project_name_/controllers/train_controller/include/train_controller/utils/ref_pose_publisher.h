#ifndef $_PROJECT_384$_REF_POSE_PUBLISHER_H
#define $_PROJECT_384$_REF_POSE_PUBLISHER_H

#include <string>
#include <vector>
#include <memory>
#include <armadillo>

#include <rviz_lib/tf_pose_publisher.h>


using namespace as64_;

class RefPosePublisher
{
public:
  RefPosePublisher(const std::string &base_link, const std::string &ref_pose_link)
  {
    started_ = false;
    P_ref = {0, 0, 0};
    Q_ref = {1, 0, 0, 0};
    auto getPose_fun = [this](){ return arma::join_vert(P_ref, Q_ref); };
    ref_pose_pub.reset(new rviz_::TfPosePublisher(getPose_fun, base_link, ref_pose_link) );
  }

  ~RefPosePublisher() { stop(); }

  void setRefPose(const arma::vec &Pd, const arma::vec &Qd) { P_ref=Pd; Q_ref=Qd; }

  void start(unsigned int pub_cycle_ms=50)
  {
    if (!started_)
    {
      ref_pose_pub->start(pub_cycle_ms);
      started_ = true;
    }
  }

  void stop() { ref_pose_pub->stop(); started_=false; }

private:
  std::shared_ptr<rviz_::TfPosePublisher> ref_pose_pub;
  arma::vec P_ref, Q_ref;
  bool started_;
};

#endif // $_PROJECT_384$_REF_POSE_PUBLISHER_H
