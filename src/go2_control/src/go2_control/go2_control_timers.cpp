/**
 * Go2 Control timer callbacks.
 *
 * September 28, 2024
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
 * @brief Timer callback to switch the LiDAR on/off.
 */
void Go2Control::utlidar_switch_timer_callback()
{
  bool use_utlidar = this->get_parameter("use_utlidar").as_bool();
  String msg{};
  if (use_utlidar) {
    msg.set__data("ON");
  } else {
    msg.set__data("OFF");
  }
  utlidar_switch_pub_->publish(msg);

  utlidar_switch_count_++;
  if (utlidar_switch_count_ >= 5) {
    utlidar_switch_timer_->cancel();
    utlidar_switch_pub_.reset();
  }
}

} // namespace go2_control
