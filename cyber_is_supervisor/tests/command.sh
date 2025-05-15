rostopic pub /robot_state std_msgs/String "data: 'START_MISSION'" --once
rostopic pub /robot_state std_msgs/String "data: 'ABORT_MISSION'" --once
rostopic pub /robot_state std_msgs/String "data: 'RESTART_MISSION'" --once

rostopic pub /leds_mode std_msgs/String "data: 'FULL_RAINBOW'" --once

rostopic pub /robot_mode std_msgs/String "data: 'AUTO'" --once

rostopic pub /robot_mode std_msgs/String "data: 'MANUAL'" --once

rostopic pub /magnet_filtered std_msgs/Bool "data: True" --once


rostopic pub /line_detector_position std_msgs/String "data: 'FULL_LINE'" --once


sudo systemctl restart ros_start.service

