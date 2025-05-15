//
// Created by victoria on 15.05.25.
//

#ifndef ZONENAVIGATOR_H
#define ZONENAVIGATOR_H

#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <string>


struct EdgeCenter {
    double x, y;
    double edge_dx, edge_dy; // kierunek krawędzi (do obliczenia prostopadłego yaw, jeśli chcesz)
};


class ZoneNavigator  {
public:
    explicit ZoneNavigator(ros::NodeHandle &nh, const std::string &state_topic);

private:
    ros::NodeHandle nh_;
    using Point = std::pair<double, double>;
    std::vector<Point> points_;

    std::string state_topic_, zone_topic_, final_orientation_;

    ros::Publisher poly_pub_, state_pub_;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;

    void publishPolygon() const;

    std::pair<double, double> centroid() const;

    static geometry_msgs::Quaternion yawToQuaternion(double yaw);

    void sendGoalAndWait();

    void publishAbort() const;

    void publishState(const std::string &msg) const;

    void computeCentersAndCentroid(std::vector<EdgeCenter>& edge_centers, std::pair<double, double>& centroid);

};


#endif //ZONENAVIGATOR_H
