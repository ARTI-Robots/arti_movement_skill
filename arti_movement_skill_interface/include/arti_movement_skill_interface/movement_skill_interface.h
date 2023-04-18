/*
Created by clemens on 28.04.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_MOVEMENT_SKILL_INTERFACE_MOVEMENT_SKILL_INTERFACE_H
#define ARTI_MOVEMENT_SKILL_INTERFACE_MOVEMENT_SKILL_INTERFACE_H

#include <arti_graph_processing/types.h>
#include <functional>
#include <ros/node_handle.h>
#include <string>

namespace arti_movement_skill_interface
{

class MovementSkillInterface
{
public:
  using ResultCB = std::function<void()>;

  virtual ~MovementSkillInterface() = default;

  virtual void initialize(
    std::string name, const ros::NodeHandle& node_handle, ResultCB finished_with_success_cb,
    ResultCB finished_with_failure_cb) = 0;

  virtual double estimateCosts(
    const arti_graph_processing::VertexPtr& source, const arti_graph_processing::VertexPtr& destination) = 0;

  virtual void performSkill(
    const arti_graph_processing::VertexPtr& source, const arti_graph_processing::VertexPtr& destination) = 0;

  virtual void stopPerformingSkill() = 0;
};

}

#endif //ARTI_MOVEMENT_SKILL_INTERFACE_MOVEMENT_SKILL_INTERFACE_H
