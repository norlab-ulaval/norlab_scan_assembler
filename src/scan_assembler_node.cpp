#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <pointmatcher/PointMatcher.h>
#include <geometry_msgs/TransformStamped.h>

typedef PointMatcher<float> PM;

std::unique_ptr<tf2_ros::Buffer> tfBuffer;
sensor_msgs::PointCloud2 firstCloud;
sensor_msgs::PointCloud2 secondCloud;
std::atomic_bool newFirstCloud;
std::atomic_bool newSecondCloud;
std::shared_ptr<PM::Transformation> transformator;

void firstCloudCallback(const sensor_msgs::PointCloud2& msg)
{
    firstCloud = msg;
    newFirstCloud.store(true);
}

void secondCloudCallback(const sensor_msgs::PointCloud2& msg)
{
    secondCloud = msg;
    newSecondCloud.store(true);
}

PM::TransformationParameters findTransform(const std::string& sourceFrame, const std::string& targetFrame, const ros::Time& time, const int& transformDimension)
{
    geometry_msgs::TransformStamped tf = tfBuffer->lookupTransform(targetFrame, sourceFrame, time, ros::Duration(0.1));
    return PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(tf, transformDimension);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_assembler_node");
    ros::NodeHandle nodeHandle;
    ros::NodeHandle privateNodeHandle("~");

    std::string robotFrame;
    privateNodeHandle.param<std::string>("robot_frame", robotFrame, "base_link");

    transformator = PM::get().TransformationRegistrar.create("RigidTransformation");

    tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer);
    tf2_ros::TransformListener tfListener(*tfBuffer);

    newFirstCloud.store(false);
    newSecondCloud.store(false);

    ros::Publisher cloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("cloud_out", 1);

    ros::Subscriber firstCloudSubscriber = nodeHandle.subscribe("cloud_in_1", 1, firstCloudCallback);
    ros::Subscriber secondCloudSubscriber = nodeHandle.subscribe("cloud_in_2", 1, secondCloudCallback);

    ros::Rate loopRate(20);
    while(ros::ok())
    {
        if(newFirstCloud.load() && newSecondCloud.load())
        {
            newFirstCloud.store(false);
            newSecondCloud.store(false);
            ros::Time lookupTime;
            if(firstCloud.header.stamp < secondCloud.header.stamp)
            {
                lookupTime = firstCloud.header.stamp;
            }
            else
            {
                lookupTime = secondCloud.header.stamp;
            }
            try
            {
                PM::TransformationParameters firstTransform = findTransform(firstCloud.header.frame_id, robotFrame,
                                                                            lookupTime, 4);
                PM::TransformationParameters secondTransform = findTransform(secondCloud.header.frame_id, robotFrame,
                                                                             lookupTime, 4);
                PM::DataPoints firstCloudPM = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(firstCloud);
                PM::DataPoints secondCloudPM = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(secondCloud);
                PM::DataPoints assembledCloudPM = transformator->compute(firstCloudPM, firstTransform);
                assembledCloudPM.concatenate(transformator->compute(secondCloudPM, secondTransform));
                sensor_msgs::PointCloud2 assembledCloud = PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(assembledCloudPM, robotFrame, lookupTime);
                cloudPublisher.publish(assembledCloud);
            }
            catch(const tf2::TransformException& ex)
            {
                ROS_WARN("%s", ex.what());
            }
        }

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
