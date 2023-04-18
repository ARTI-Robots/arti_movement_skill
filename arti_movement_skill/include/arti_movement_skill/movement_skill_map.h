/*
Created by abuchegger on 2020-11-27.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ARTI_MOVEMENT_SKILL_MOVEMENT_SKILL_MAP_H
#define ARTI_MOVEMENT_SKILL_MOVEMENT_SKILL_MAP_H

#include <boost/shared_ptr.hpp>
#include <arti_movement_skill_interface/movement_skill_interface.h>
#include <map>
#include <pluginlib/class_loader.h>
#include <ros/node_handle.h>
#include <string>

namespace arti_movement_skill
{

class MovementSkillMap
{
public:
  using MovementSkillPtr = boost::shared_ptr<arti_movement_skill_interface::MovementSkillInterface>;

  void load(
    const ros::NodeHandle& node_handle,
    const arti_movement_skill_interface::MovementSkillInterface::ResultCB& finished_with_success_cb,
    const arti_movement_skill_interface::MovementSkillInterface::ResultCB& finished_with_failure_cb,
    bool use_mockup);

  MovementSkillPtr at(const std::string& key) const;
  MovementSkillPtr find(const std::string& key) const;

protected:
  pluginlib::ClassLoader<arti_movement_skill_interface::MovementSkillInterface> plugin_loader_{
    "arti_movement_skill_interface", "arti_movement_skill_interface::MovementSkillInterface"};

  std::map<std::string, MovementSkillPtr> map_;
};

}

#endif //ARTI_MOVEMENT_SKILL_MOVEMENT_SKILL_MAP_H
