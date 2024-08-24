/**
 * Go2 Control node service callbacks.
 *
 * August 6, 2024
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <go2_control/go2_control.hpp>

namespace go2_control
{

/**
 * @brief Tells the robot to perform a special action.
 *
 * @param req Service request.
 * @param resp Service response.
 */
void Go2Control::actions_callback(
  Action::Request::SharedPtr req,
  Action::Response::SharedPtr resp)
{
  Request action_req{};
  action_req.header.identity.set__id(this->get_clock()->now().nanoseconds());
  action_req.header.identity.set__api_id(req->action.id);

  sport_request_pub_->publish(action_req);

  // Wait for the action to be completed by simply waiting
  std::this_thread::sleep_for(std::chrono::milliseconds(actions_sleep_time_));

  resp->set__success(true);
}

/**
 * @brief Sets obstacle avoidance service state.
 *
 * @param req Service request.
 * @param resp Service response.
 */
void Go2Control::obstacle_avoidance_callback(
  SetBool::Request::SharedPtr req,
  SetBool::Response::SharedPtr resp)
{
  set_obstacle_avoidance(req->data);

  resp->success = true;
  resp->message = "";
}

} // namespace go2_control
