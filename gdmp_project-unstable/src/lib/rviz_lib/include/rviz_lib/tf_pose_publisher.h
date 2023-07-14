
#ifndef AS64_TF_POSE_PUBLISHER_H
#define AS64_TF_POSE_PUBLISHER_H

#include <cstdlib>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>

#include <armadillo>

namespace as64_
{

namespace rviz_
{

/**
  \brief Used to publish a pose as a tf transform in rviz.
         Publishing takes place in a separate thread.
         The publish rate can be set in milliseconds.
 */
class TfPosePublisher
{
public:

  /**
    \brief Constructor.
    @param[in] getPose: function that returns a 7x1 vector where the first 3 elements are the position and the last for the orientation as unit quaternion.
    @param[in] parent_link: the reference urdf link wrt which the pose is expressed.
    @param[in] child_link: The name of the published tf.
   */
  TfPosePublisher(std::function<arma::vec()> getPose, const std::string &parent_link, const std::string child_link="pose");

  /**
    \brief Starts a thread that publishes the tf-transform to rviz.
    @param[in] pub_rate_ms: The publish rate in milliseconds. (optional, default=200)
   */
  void start(unsigned pub_rate_ms = 200);

  /**
    \brief Stops the thread that publishes the tf-transform to rviz.
   */
  void stop();

  /**
    \brief Sets the publish rate.
    @param[in] pub_rate_ms: The publish rate in milliseconds.
   */
  void setPublishRate(unsigned pub_rate_ms);

private:

  void setPublish(bool set);

  unsigned pub_rate_ms_;

  std::function<arma::vec()> get_pose_fun_; // return the pose as [pos; quat]

  bool publish_;
  std::mutex pub_mutex;

  std::string parent_link_;
  std::string child_link_;
};


} // namespace rviz_

} // namespace as64_

#endif // AS64_TF_POSE_PUBLISHER_H
