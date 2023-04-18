/*
Created by abuchegger on 2020-11-26.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <arti_movement_skill/mock_movement_skill.h>
#include <arti_graph_processing/vertex.h>
#include <utility>

namespace arti_movement_skill
{

double MockMovementSkill::estimateCosts(
  const arti_graph_processing::VertexPtr& source,  const arti_graph_processing::VertexPtr& destination)
{
  return source->calculateEuclideanDistanceTo(*destination);
}

void MockMovementSkill::performSkill(
  const arti_graph_processing::VertexPtr& source,  const arti_graph_processing::VertexPtr& destination)
{
  if (active_)
  {
    ROS_WARN_STREAM("performSkill called while skill already active");
    timer_.stop();
  }

  ROS_INFO_STREAM(
    "Starting to mock '" << name_ << "' movement skill from vertex '" << source->getName() << "' to '"
                         << destination->getName() << "'");
  active_ = true;
  timer_ = node_handle_.createTimer(ros::Duration(2.0), &MockMovementSkill::processTimerExpiration, this, true);
}

void MockMovementSkill::stopPerformingSkill()
{
  if (!active_)
  {
    ROS_WARN_STREAM("stopPerformingSkill called while skill inactive");
  }

  active_ = false;
  timer_.stop();
}

void MockMovementSkill::processTimerExpiration(const ros::TimerEvent& /*timer_event*/)
{
  if (active_)
  {
    ROS_INFO_STREAM("Finished mocking '" << name_ << "' movement skill");
    active_ = false;
    if (finished_with_success_cb_)
    {
      finished_with_success_cb_();
    }
  }
}

}
