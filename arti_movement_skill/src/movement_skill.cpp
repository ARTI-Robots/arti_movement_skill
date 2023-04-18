/*
Created by clemens on 28.04.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_movement_skill/movement_skill.h>
#include <arti_graph_processing/a_star_algorithm.h>
#include <arti_graph_processing/graph.h>
#include <arti_ros_param/arti_ros_param.h>
#include <functional>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace arti_movement_skill
{
MovementSkill::MovementSkill(const ros::NodeHandle& nh)
  : nh_(nh),
    odom_subscriber_(nh_.subscribe("/odom", 1, &MovementSkill::processOdom, this)), tf_listener_(tf_buffer_),
    movement_graph_publisher_(nh_, "movement_graph"), action_server_(nh_, "movement_skill", false)
{
  movement_skill_map_.load(ros::NodeHandle(nh_, "movement_skills"),
                           std::bind(&MovementSkill::processMovementSkillSuccess, this),
                           std::bind(&MovementSkill::processMovementSkillFailure, this),
                           nh_.param<bool>("use_mock_movement_skill", false));

  GraphLoader graph_loader{movement_skill_map_};
  movement_graph_ = graph_loader.loadGraph(arti_ros_param::RootParam(ros::NodeHandle{nh_, "graph"}));

  if (movement_graph_)
  {
    movement_graph_publisher_.publish(*movement_graph_);
  }

  action_server_.registerGoalCallback(std::bind(&MovementSkill::processNewMovementSkillActionGoal, this));
  action_server_.registerPreemptCallback(std::bind(&MovementSkill::processMovementSkillActionPreemption, this));
  action_server_.start();
}

void MovementSkill::processOdom(const nav_msgs::OdometryConstPtr& odom)
{
  robot_pose_.header = odom->header;
  robot_pose_.pose = odom->pose.pose;
}

void MovementSkill::processNewMovementSkillActionGoal()
{
  const arti_movement_skill_msgs::MovementSkillGoalConstPtr goal = action_server_.acceptNewGoal();

  resetPlan();

  if (!movement_graph_)
  {
    ROS_ERROR_STREAM("movement graph is not set");
    action_server_.setAborted({}, "movement graph is not set");
    return;
  }

  if (robot_pose_.header.stamp.isZero())
  {
    ROS_ERROR_STREAM("robot position is unknown");
    action_server_.setAborted({}, "robot position is unknown");
    return;
  }

  geometry_msgs::PoseStamped current_pose;
  try
  {
    current_pose = tf_buffer_.transform(robot_pose_, movement_graph_->getFrameName(), ros::Duration(1.));
  }
  catch (const std::exception& exception)
  {
    ROS_ERROR_STREAM("robot pose can not be transformed: " << exception.what());
    action_server_.setAborted({}, "robot pose can not be transformed");
    return;
  }

  const arti_graph_processing::VertexPtr start_vertex = movement_graph_->getClosestVertex(current_pose);
  if (!start_vertex)
  {
    ROS_ERROR_STREAM("start vertex can not be found");
    action_server_.setAborted({}, "start vertex cannot be found");
    return;
  }

  const arti_graph_processing::VertexPtr goal_vertex = movement_graph_->getVertex(goal->target_name);
  if (!goal_vertex)
  {
    ROS_ERROR_STREAM("goal vertex '" << goal->target_name << "' cannot be found");
    action_server_.setAborted({}, "goal vertex cannot be found");
    return;
  }

  if (goal_vertex == start_vertex)
  {
    ROS_INFO_STREAM("goal vertex is the same as start vertex, so nothing is to be done");
    arti_movement_skill_msgs::MovementSkillResult result;
    result.goal_reached = true;
    action_server_.setSucceeded(result, "goal vertex is the same as start vertex, so nothing is to be done.");
    return;
  }

  std::vector<std::pair<arti_graph_processing::VertexPtr,
    std::shared_ptr<arti_graph_processing::Edge>>> path
    = arti_graph_processing::AStarAlgorithm::computePath(start_vertex, goal_vertex);

  for (const auto& path_segment : path)
  {
    const auto edge = std::dynamic_pointer_cast<Edge>(path_segment.second);
    if (edge)
    {
      plan_.push(edge);
    }
    else if ((plan_.size() + 1) != path.size())
    {
      // The last segment's edge can be null (represents last vertex), but all others must be valid.
      ROS_ERROR_STREAM("movement skill graph search returned null or invalid edge");
      action_server_.setAborted({}, "movement skill graph search returned null or invalid edge");
      return;
    }
  }

  processPath();
}

void MovementSkill::processMovementSkillActionPreemption()
{
  if (current_movement_skill_)
  {
    current_movement_skill_->stopPerformingSkill();
    current_movement_skill_.reset();
  }

  resetPlan();

  action_server_.setPreempted({}, "This goal was canceled as it was requested by the action client");
}

void MovementSkill::processMovementSkillSuccess()
{
  current_movement_skill_.reset();

  if (action_server_.isActive())
  {
    if (plan_.empty())
    {
      arti_movement_skill_msgs::MovementSkillResult result;
      result.goal_reached = true;
      action_server_.setSucceeded(result, "Goal reached.");
    }
    else
    {
      processPath();
    }
  }
}

void MovementSkill::processMovementSkillFailure()
{
  current_movement_skill_.reset();

  if (action_server_.isActive())
  {
    resetPlan();

    arti_movement_skill_msgs::MovementSkillResult result;
    result.goal_reached = false;
    action_server_.setSucceeded(result, "Goal not reached.");
  }
}

void MovementSkill::processPath()
{
  if (!plan_.empty())
  {
    const auto edge = plan_.front();
    plan_.pop();

    current_movement_skill_ = movement_skill_map_.at(edge->getType());
    current_movement_skill_->performSkill(edge->getSource(), edge->getDestination());
  }
}

void MovementSkill::resetPlan()
{
  Plan{}.swap(plan_);  // Clear plan
  current_movement_skill_.reset();
}


MovementSkill::GraphLoader::GraphLoader(MovementSkillMap& movement_skill_map)
  : movement_skill_map_(movement_skill_map)
{
}

arti_graph_processing::EdgePtr MovementSkill::GraphLoader::loadEdge(
  const arti_graph_processing::Graph& /*graph*/, const arti_graph_processing::VertexPtr& source,
  const arti_graph_processing::VertexPtr& destination, double /*costs*/, const arti_ros_param::Param& root_param)
{
  const auto type = root_param["type"].decode<std::string>();
  if (!type)
  {
    root_param.handleTypeError("required parameter 'type' is missing");
    return {};
  }

  const auto movement_skill = movement_skill_map_.find(type.value());
  if (!movement_skill)
  {
    root_param.handleTypeError("edge has type '" + type.value() + "' which is not in movement_skills");
    return {};
  }

  return std::make_shared<Edge>(source, destination, movement_skill->estimateCosts(source, destination), type.value());
}

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "arti_movement_skill");

  ros::NodeHandle private_nh("~");

  arti_movement_skill::MovementSkill movement_skill(private_nh);

  ros::spin();
  return 0;
}
