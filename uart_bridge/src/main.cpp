#include <cstdio>
#include <cstdlib>
#include "uart_bridge/serial_slip.h"
#include "uart_bridge/rosTopic.h"
#include "ros/ros.h"
#include <string>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/BatteryState.h"
#include "nav_msgs/Odometry.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include <cmath>

SerialSlip *slip = nullptr;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    CmdVelMsg_t packet = {0};
    packet.msgID = CMD_VEL_MSG;

    // Przepisz wartości z ROS na strukturę
    packet.linear.x = msg->linear.x;
    packet.linear.y = msg->linear.y;
    packet.linear.z = msg->linear.z;
    packet.angular.x = msg->angular.x;
    packet.angular.y = msg->angular.y;
    packet.angular.z = msg->angular.z;

    int result = serial_slip_write(slip, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    if (result < 0) {
        ROS_ERROR("serial_slip_write failed");
    }
}

void ledsCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    LEDMsg_t packet = {0};
    packet.msgID = LED_MSG;

    for (size_t i = 0; i < 12; i++) {
        size_t base = i * 4;
        packet.leds[i].R = msg->data[base];
        packet.leds[i].G = msg->data[base + 1];
        packet.leds[i].B = msg->data[base + 2];
        packet.leds[i].A = msg->data[base + 3];
    }
    int result = serial_slip_write(slip, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    if (result < 0) {
        ROS_ERROR("serial_slip_write failed");
    }
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    PoseMsg_t packet = {0};
    packet.msgID = POSE_MSG;

    packet.x = msg->position.x;
    packet.y = msg->position.y;
    const auto q = msg->orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    packet.yaw = std::atan2(siny_cosp, cosy_cosp);

    int result = serial_slip_write(slip, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    if (result < 0) {
        ROS_ERROR("serial_slip_write failed");
    }
}


void publishPackageStatus(ros::Publisher &pub, uint8_t level, const std::string &name, const std::string &message) {
    diagnostic_msgs::DiagnosticStatus status;
    status.level = level; // 0 = OK, 1 = WARN, 2 = ERROR
    status.name = name;
    status.message = message;
    status.hardware_id = "uart_bridge";

    diagnostic_msgs::KeyValue kv;
    kv.key = "Timestamp";
    kv.value = std::to_string(ros::Time::now().toSec());
    status.values.push_back(kv);

    pub.publish(status);
}


int main(int argc, char *argv[]) {
    uint64_t counter=0;
    ros::init(argc, argv, "uart_bridge2");
    ros::NodeHandle nh("~");

    std::string uart_port;
    int frequency;
    std::string twist_topic, pose_topic, odom_topic, imu_topic, magnet_topic, line_detector_topic, leds_topic, battery_topic, status_topic;
    std::string odom_frame, imu_frame;

    nh.param<std::string>("uart_port", uart_port, "/dev/ttyACM0");
    nh.param<int>("frequency", frequency, 100);

    nh.param<std::string>("twist_topic", twist_topic, "/cmd_vel");

    nh.param<std::string>("pose_topic", pose_topic, "/slam_out_pose");

    nh.param<std::string>("odom_topic", odom_topic, "/low_level_odom");
    nh.param<std::string>("odom_frame", odom_frame, "map");

    nh.param<std::string>("imu_topic", imu_topic, "/imu");
    nh.param<std::string>("imu_frame", imu_frame, "imu");


    nh.param<std::string>("magnet_topic", magnet_topic, "/magnet");
    nh.param<std::string>("line_detector_topic", line_detector_topic, "/line_detector");
    nh.param<std::string>("leds_topic", leds_topic, "/leds");
    nh.param<std::string>("battery_topic", battery_topic, "/battery");
    nh.param<std::string>("status_topic", status_topic, "/status");



    ros::Subscriber sub_cmd_vel = nh.subscribe(twist_topic, 10, cmdVelCallback);
    ros::Subscriber sub_lights = nh.subscribe(leds_topic, 10, ledsCallback);
    ros::Subscriber sub_pose = nh.subscribe(pose_topic, 10, poseCallback);

    ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);
    ros::Publisher imu_publisher = nh.advertise<sensor_msgs::Imu>(imu_topic, 10);
    ros::Publisher magnet_publisher = nh.advertise<std_msgs::Float64MultiArray>(magnet_topic, 10);
    ros::Publisher line_detector_publisher = nh.advertise<std_msgs::UInt16MultiArray>(line_detector_topic, 10);
    ros::Publisher battery_publisher = nh.advertise<sensor_msgs::BatteryState>(battery_topic, 10);
    ros::Publisher status_publisher = nh.advertise<diagnostic_msgs::DiagnosticStatus>(status_topic, 10);

    ros::spinOnce(); // pozwala ROS-owi przetworzyć połączenia
    ros::Duration(0.1).sleep();
    publishPackageStatus(status_publisher, diagnostic_msgs::DiagnosticStatus::OK, "ROS STARTED", "Ros read params, prepare subscribers and publishers");


    slip = serial_slip_open(uart_port.c_str());
    if (!slip) {
        publishPackageStatus(status_publisher, diagnostic_msgs::DiagnosticStatus::ERROR, "FAILED SERIAL OPENED", "Failed to open serial slip");
        return 1;
    } else {
        publishPackageStatus(status_publisher, diagnostic_msgs::DiagnosticStatus::OK, "SERIAL OPENED", "Serial slip opened");
    }
    uint8_t recvBuffer[8192];
    size_t recvLen = 0;


    ros::Time last_diag_time = ros::Time::now();
    ros::Rate loop_rate(frequency);
    publishPackageStatus(status_publisher, diagnostic_msgs::DiagnosticStatus::OK, "READY", "Start ros loop");

    while (ros::ok()) {
        if (SS_OK == serial_slip_get_message(slip, recvBuffer, sizeof(recvBuffer), &recvLen, 0)) {
            switch ((msgID_t) recvBuffer[0]) {
                case IMU_MSG: {
                    ImuMsg_t *imuMsg = (ImuMsg_t *) recvBuffer;
                    sensor_msgs::Imu ros_imu;

                    ros_imu.header.stamp = ros::Time::now();
                    ros_imu.header.frame_id = imu_frame;

                    ros_imu.orientation.x = imuMsg->orientation.x;
                    ros_imu.orientation.y = imuMsg->orientation.y;
                    ros_imu.orientation.z = imuMsg->orientation.z;
                    ros_imu.orientation.w = imuMsg->orientation.w;

                    for (int i = 0; i < 9; i++) {
                        ros_imu.orientation_covariance[i] = 0.0;
                        ros_imu.angular_velocity_covariance[i] = 0.0;
                        ros_imu.linear_acceleration_covariance[i] = 0.0;
                    }

                    ros_imu.angular_velocity.x = imuMsg->angular_velocity.x;
                    ros_imu.angular_velocity.y = imuMsg->angular_velocity.y;
                    ros_imu.angular_velocity.z = imuMsg->angular_velocity.z;

                    ros_imu.linear_acceleration.x = imuMsg->linear_acceleration.x;
                    ros_imu.linear_acceleration.y = imuMsg->linear_acceleration.y;
                    ros_imu.linear_acceleration.z = imuMsg->linear_acceleration.z;
                    imu_publisher.publish(ros_imu);
                    break;
                }
                case TRACKER_SENSOR_MSG: {
                    TrackerSensorMsg_t *trackerMsg = (TrackerSensorMsg_t *) recvBuffer;
                    std_msgs::UInt16MultiArray ros_msg;

                    for (int i = 0; i < 5; ++i) {
                        ros_msg.data.push_back(trackerMsg->data[i]);
                    }

                    ros_msg.layout.dim.resize(1);
                    ros_msg.layout.dim[0].label = "lines";
                    ros_msg.layout.dim[0].size = 5;
                    ros_msg.layout.dim[0].stride = 5;
                    ros_msg.layout.data_offset = 0;

                    line_detector_publisher.publish(ros_msg);
                    break;
                }
                case BATTERY_STATE_MSG: {
                    BatteryStateMsg_t *batteryMsg = (BatteryStateMsg_t *) recvBuffer;
                    sensor_msgs::BatteryState battery;

                    battery.header.stamp = ros::Time::now();
                    battery.voltage = batteryMsg->voltage; // [V]
                    battery.current = batteryMsg->current; // [A]
                    battery.charge = static_cast<float>(batteryMsg->energy);
                    battery.percentage = batteryMsg->percentage / 100.0f;

                    battery.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
                    battery.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
                    battery.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
                    battery.present = true;

                    battery_publisher.publish(battery);
                    break;
                }
                case MAGNET_MSG: {
                    MagnetMsg_t *magnetMsg = (MagnetMsg_t *) recvBuffer;
                    std_msgs::Float64MultiArray magnet_array;

                    magnet_array.data.push_back(magnetMsg->magnet.x);
                    magnet_array.data.push_back(magnetMsg->magnet.y);
                    magnet_array.data.push_back(magnetMsg->magnet.z);

                    magnet_publisher.publish(magnet_array);
                    break;
                }
                case ODOMETRY_MSG: {
                    OdometryMsg_t *odomMsg = (OdometryMsg_t *) recvBuffer;
                    nav_msgs::Odometry odom;

                    odom.header.stamp = ros::Time::now();
                    odom.header.seq = odomMsg->header.seq;
                    odom.header.frame_id = odom_frame;

                    odom.pose.pose.position.x = odomMsg->position.x;
                    odom.pose.pose.position.y = odomMsg->position.y;
                    odom.pose.pose.position.z = odomMsg->position.z;

                    odom.pose.pose.orientation.x = odomMsg->orientation.x;
                    odom.pose.pose.orientation.y = odomMsg->orientation.y;
                    odom.pose.pose.orientation.z = odomMsg->orientation.z;
                    odom.pose.pose.orientation.w = odomMsg->orientation.w;

                    odom.twist.twist.linear.x = odomMsg->linear.x;
                    odom.twist.twist.linear.y = odomMsg->linear.y;
                    odom.twist.twist.linear.z = odomMsg->linear.z;

                    odom.twist.twist.angular.x = odomMsg->angular.x;
                    odom.twist.twist.angular.y = odomMsg->angular.y;
                    odom.twist.twist.angular.z = odomMsg->angular.z;

                    odom_publisher.publish(odom);
                    break;
                }
                case DIAG_MSG: {
                    DiagMsg_t *diagMsg = (DiagMsg_t *) recvBuffer;
                    std::string name(reinterpret_cast<const char*>(diagMsg->name), diagMsg->nameLen);
                    std::string message(reinterpret_cast<const char*>(diagMsg->msg), diagMsg->msgLen);

                    diagnostic_msgs::DiagnosticStatus status;
                    status.level = diagMsg->level; // 0 = OK, 1 = WARN, 2 = ERROR
                    status.name = name;
                    status.message = message;
                    status.hardware_id = "low_level_board";

                    diagnostic_msgs::KeyValue kv;
                    kv.key = "Timestamp";
                    kv.value = std::to_string(ros::Time::now().toSec());
                    status.values.push_back(kv);

                    status_publisher.publish(status);
                    break;
                }
                default:
                    break;
            }
        }
        if ((ros::Time::now() - last_diag_time).toSec() >= 1.0) {
            counter++;
            publishPackageStatus(status_publisher, diagnostic_msgs::DiagnosticStatus::OK, "PACKAGE", "WORK: "+std::to_string(counter));
            last_diag_time = ros::Time::now();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
