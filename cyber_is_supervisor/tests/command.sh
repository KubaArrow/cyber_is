rostopic pub /robot_state std_msgs/String "data: 'START_MISSION'" --once
rostopic pub /robot_state std_msgs/String "data: 'ABORT_MISSION'" --once

rostopic pub /robot_mode std_msgs/String "data: 'AUTO'" --once

rostopic pub /robot_mode std_msgs/String "data: 'MANUAL'" --once


