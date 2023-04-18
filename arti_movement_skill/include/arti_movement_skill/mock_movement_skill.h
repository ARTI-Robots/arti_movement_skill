/*
Created by abuchegger on 2020-11-26.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ARTI_MOVEMENT_SKILL_MOCK_MOVEMENT_SKILL_H
#define ARTI_MOVEMENT_SKILL_MOCK_MOVEMENT_SKILL_H

#include <arti_movement_skill_interface/abstract_movement_skill.h>
#include <ros/node_handle.h>
#include <ros/timer.h>
#include <string>

namespace arti_movement_skill
{

class MockMovementSkill : public arti_movement_skill_interface::AbstractMovementSkill
{
public:
  double estimateCosts(
    const arti_graph_processing::VertexPtr& source, const arti_graph_processing::VertexPtr& destination) override;

  void performSkill(
    const arti_graph_processing::VertexPtr& source, const arti_graph_processing::VertexPtr& destination) override;

  void stopPerformingSkill() override;

protected:
  void processTimerExpiration(const ros::TimerEvent& timer_event);

  ros::Timer timer_;
  bool active_{false};
};

}

#endif //ARTI_MOVEMENT_SKILL_MOCK_MOVEMENT_SKILL_H
