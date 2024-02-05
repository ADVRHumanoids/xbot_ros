#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include <eigen_conversions/eigen_msg.h>

#include <xbot2_interface/collision.h>
#include <xbot2_interface/ros/config_from_param.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_collision_publisher_node");

    ros::NodeHandle n, npr("~");

    double detection_th = npr.param("detection_th", -1.0);

    std::string fixed_frame = "base_link";
    npr.getParam("fixed_frame", fixed_frame);

    bool verbose = npr.param<bool>("verbose", false);

    auto cfg = XBot::Utils::ConfigOptionsFromParamServer(n);

    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(cfg);

    XBot::Collision::CollisionModel cm(model);

    auto pub = n.advertise<visualization_msgs::Marker>("collision_marker", 1);

    auto coll_pairs = cm.getCollisionPairs();

    auto js_callback = [&](const sensor_msgs::JointStateConstPtr& msg)
    {
        if(verbose)
        {
            ROS_INFO("got msg");
        }

        for(int i = 0; i < msg->name.size(); i++)
        {
            model->getJoint(msg->name[i])->setJointPositionMinimal(msg->position[i]);
        }

        model->update();

        auto tic = ros::WallTime::now();
        cm.update();
        auto d = cm.computeDistance(detection_th);
        auto toc = ros::WallTime::now();

        ROS_INFO_THROTTLE(1, "distance computation took %f ms", 1000*(toc-tic).toSec());

        auto wp_vector = cm.getWitnessPoints();

        visualization_msgs::Marker marker;
        marker.header = msg->header;
        marker.header.frame_id = fixed_frame;
        marker.type = marker.LINE_LIST;

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.pose.orientation.w = 1;

        marker.scale.x = 0.005;

        auto f_T_w = model->getPose(fixed_frame).inverse();

        int i = 0;

        for(int i = 0; i < coll_pairs.size(); i++)
        {
            geometry_msgs::Point gp1, gp2;

            Eigen::Vector3d p1 = f_T_w * wp_vector[i].first;
            Eigen::Vector3d p2 = f_T_w * wp_vector[i].second;

            if(detection_th > 0 && d[i] > detection_th)
            {
                continue;
            }

            gp1.x = p1.x();
            gp1.y = p1.y();
            gp1.z = p1.z();

            gp2.x = p2.x();
            gp2.y = p2.y();
            gp2.z = p2.z();

            marker.points.push_back(gp1);
            marker.points.push_back(gp2);

            if(verbose && d[i] > 0)
            {
                ROS_INFO("%s vs %s: d = %f", coll_pairs[i].first.c_str(), coll_pairs[i].second.c_str(), d[i]);
            }

            if(verbose && d[i] < 0)
            {
                ROS_WARN("%s vs %s: d = %f", coll_pairs[i].first.c_str(), coll_pairs[i].second.c_str(), d[i]);
            }
        }

        pub.publish(marker);
    };

    auto sub = n.subscribe<sensor_msgs::JointState>("joint_states", 1, js_callback);

    ROS_INFO("detection threshold is %f", detection_th);

    ROS_INFO("number of collision pairs is %ld", coll_pairs.size());

    ROS_INFO("subscribed to %s", sub.getTopic().c_str());

    ROS_INFO("spinning");

    ros::spin();


}
