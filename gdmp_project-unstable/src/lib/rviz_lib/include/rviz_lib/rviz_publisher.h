#ifndef RVIZ_VISUAL_TOOLS_PUBLISHER_H
#define RVIZ_VISUAL_TOOLS_PUBLISHER_H

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <armadillo>

namespace as64_
{

namespace rviz_
{


class RvizPublisher
{
public:

  RvizPublisher(const std::string &world_frame, const std::string &publish_topic);

  void publishPath(const arma::mat &path, rviz_visual_tools::colors color=rviz_visual_tools::BLUE,
      rviz_visual_tools::scales scale=rviz_visual_tools::MEDIUM);

  void publishPath(const std::vector<arma::vec> &path, rviz_visual_tools::colors color=rviz_visual_tools::BLUE,
      rviz_visual_tools::scales scale=rviz_visual_tools::MEDIUM);

  void publishOrientedPath(const arma::mat &Pos, const arma::mat &Quat, const std::vector<int> orient_frames_ind,
      rviz_visual_tools::colors color=rviz_visual_tools::BLUE, rviz_visual_tools::scales scale=rviz_visual_tools::MEDIUM);

  void publishOrientedPath(const arma::mat &Pos, const arma::mat &Quat, int n_o,
      rviz_visual_tools::colors color=rviz_visual_tools::BLUE, rviz_visual_tools::scales scale=rviz_visual_tools::MEDIUM);

  void publishPoint(const arma::vec &pos, rviz_visual_tools::colors color=rviz_visual_tools::BLUE,
                   rviz_visual_tools::scales scale=rviz_visual_tools::MEDIUM);

  void publishFrame(const arma::vec &pos, const arma::vec &quat, rviz_visual_tools::scales scale=rviz_visual_tools::MEDIUM);

  void drawnow();

  void deleteAll();

private:

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  std::vector<int> getIndices(const arma::mat &Pos, int n_ind);

};

} // namespace rviz_

} // namespace as64_

#endif
