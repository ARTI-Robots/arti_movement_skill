/*
Created by clemens on 28.04.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_MOVEMENT_SKILL_MOVEMENT_SKILL_H
#define ARTI_MOVEMENT_SKILL_MOVEMENT_SKILL_H

#include <actionlib/server/simple_action_server.h>
#include <arti_graph_processing/graph_loader.h>
#include <arti_graph_processing/graph_visualization_publisher.h>
#include <arti_graph_processing/types.h>
#include <arti_movement_skill/edge.h>
#include <arti_movement_skill/movement_skill_map.h>
#include <arti_movement_skill_msgs/MovementSkillAction.h>
#include <arti_ros_param/forward_declarations.h>
#include <geometry_msgs/PointStamped.h>
#include <map>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/node_handle.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace arti_movement_skill
{
class MovementSkill
{
public:
  explicit MovementSkill(const ros::NodeHandle& nh);

private:
  using Plan = std::queue<std::shared_ptr<Edge>>;

  struct GraphLoader : public arti_graph_processing::GraphLoader
  {
    explicit GraphLoader(MovementSkillMap& movement_skill_map);

    arti_graph_processing::EdgePtr loadEdge(
      const arti_graph_processing::Graph& graph, const arti_graph_processing::VertexPtr& source,
      const arti_graph_processing::VertexPtr& destination, double costs,
      const arti_ros_param::Param& root_param) override;

    MovementSkillMap& movement_skill_map_;
  };

  void processOdom(const nav_msgs::OdometryConstPtr& odom);

  void processNewMovementSkillActionGoal();

  void processMovementSkillActionPreemption();

  void processMovementSkillSuccess();

  void processMovementSkillFailure();

  void processPath();

  void resetPlan();

  ros::NodeHandle nh_;
  MovementSkillMap movement_skill_map_;
  arti_graph_processing::GraphPtr movement_graph_;

  ros::Subscriber odom_subscriber_;
  geometry_msgs::PoseStamped robot_pose_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  arti_graph_processing::GraphVisualizationPublisher movement_graph_publisher_;

  actionlib::SimpleActionServer<arti_movement_skill_msgs::MovementSkillAction> action_server_;

  Plan plan_;
  MovementSkillMap::MovementSkillPtr current_movement_skill_;
};
}

#endif //ARTI_MOVEMENT_SKILL_MOVEMENT_SKILL_H
