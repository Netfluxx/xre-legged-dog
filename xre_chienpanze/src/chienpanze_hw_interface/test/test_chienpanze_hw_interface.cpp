// // Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
// // Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

// #include <gmock/gmock.h>

// #include <string>

// #include "hardware_interface/resource_manager.hpp"
// #include "ros2_control_test_assets/components_urdfs.hpp"
// #include "ros2_control_test_assets/descriptions.hpp"

// class TestChienpanzeHardwareInterface : public ::testing::Test
// {
// protected:
//   void SetUp() override
//   {
//     // TODO(anyone): Extend this description to your robot
//     chienpanze_hardware_interface =
//       R"(
//         <ros2_control name="ChienpanzeHardwareInterface" type="system">
//           <hardware>
//             <plugin>chienpanze_hardware_interface/ChienpanzeHardwareInterface</plugin>
//           </hardware>
//           <joint name="joint1">
//             <command_interface name="position"/>
//             <state_interface name="position"/>
//             <param name="initial_position">1.57</param>
//           </joint>

//         </ros2_control>
//     )";

//           <joint name="upper_leg_joint">
//         <command_interface name="position">
//           <param name="min">{-2*pi}</param>
//           <param name="max">{2*pi}</param>
//         </command_interface>

//         <state_interface name="position">
//           <param name="initial_value">0.0</param>
//         </state_interface>

//       </joint>
//   }

//   std::string chienpanze_hardware_interface;
// };

// TEST_F(TestRRBotHardwareInterface, load_rrbot_hardware_interface_2dof)
// {
//   auto urdf = ros2_control_test_assets::urdf_head + chienpanze_hardware_interface +
//               ros2_control_test_assets::urdf_tail;
//   ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
// }