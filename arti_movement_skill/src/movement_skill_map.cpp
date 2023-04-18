/*
Created by abuchegger on 2020-11-27.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <arti_movement_skill/movement_skill_map.h>
#include <arti_movement_skill/mock_movement_skill.h>
#include <boost/make_shared.hpp>
#include <stdexcept>
#include <xmlrpcpp/XmlRpcValue.h>

namespace arti_movement_skill
{

void MovementSkillMap::load(
  const ros::NodeHandle& node_handle,
  const arti_movement_skill_interface::MovementSkillInterface::ResultCB& finished_with_success_cb,
  const arti_movement_skill_interface::MovementSkillInterface::ResultCB& finished_with_failure_cb, bool use_mockup)
{
  XmlRpc::XmlRpcValue movement_skill_map_config;
  node_handle.getParam("", movement_skill_map_config);

  if (movement_skill_map_config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    throw std::invalid_argument("movement skill map config has invalid type");
  }

  for (const auto& movement_skill_config : movement_skill_map_config)
  {
    if (movement_skill_config.second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      throw std::invalid_argument("movement skill config has invalid type");
    }

    const ros::NodeHandle movement_skill_node_handle{node_handle, movement_skill_config.first};

    MovementSkillPtr movement_skill;
    if (use_mockup)
    {
      movement_skill = boost::make_shared<MockMovementSkill>();
    }
    else
    {
      const auto type = movement_skill_node_handle.param<std::string>("type", "");
      try
      {
        movement_skill = plugin_loader_.createInstance(type);
      }
      catch (const pluginlib::PluginlibException& ex)
      {
        ROS_ERROR_STREAM("failed to load the '" << type << "' skill: " << ex.what());
        throw;
      }
    }

    movement_skill->initialize(movement_skill_config.first, movement_skill_node_handle,
                               finished_with_success_cb, finished_with_failure_cb);
    map_.emplace(movement_skill_config.first, movement_skill);
  }
}

boost::shared_ptr<arti_movement_skill_interface::MovementSkillInterface> MovementSkillMap::at(
  const std::string& key) const
{
  const auto it = map_.find(key);
  if (it != map_.end())
  {
    return it->second;
  }
  throw std::out_of_range("no movement skill named " + key);
}

boost::shared_ptr<arti_movement_skill_interface::MovementSkillInterface> MovementSkillMap::find(
  const std::string& key) const
{
  const auto it = map_.find(key);
  if (it != map_.end())
  {
    return it->second;
  }
  return {};
}

}
