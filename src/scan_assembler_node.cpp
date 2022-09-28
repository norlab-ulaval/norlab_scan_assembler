#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <pointmatcher/PointMatcher.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

typedef PointMatcher<float> PM;

std::unique_ptr<tf2_ros::Buffer> tfBuffer;
sensor_msgs::msg::PointCloud2 firstCloud;
sensor_msgs::msg::PointCloud2 secondCloud;
std::atomic_bool newFirstCloud;
std::atomic_bool newSecondCloud;

class ScanAssemblerNode : public rclcpp::Node
{
public:
    ScanAssemblerNode():
            Node("scan_assembler_node")
    {
    }

    void firstCloudCallback(const sensor_msgs::msg::PointCloud2& msg)
    {
        firstCloud = msg;
        newFirstCloud.store(true);
    }

    void secondCloudCallback(const sensor_msgs::msg::PointCloud2& msg)
    {
        secondCloud = msg;
        newSecondCloud.store(true);
    }
};

PM::TransformationParameters
findTransform(const std::string& sourceFrame, const std::string& targetFrame, const rclcpp::Time& time, const int& transformDimension)
{
    geometry_msgs::msg::TransformStamped tf = tfBuffer->lookupTransform(targetFrame, sourceFrame, time, std::chrono::milliseconds(100));
    return PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(tf, transformDimension);
}

int main(int argc, char**argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<ScanAssemblerNode> node = std::shared_ptr<ScanAssemblerNode>(new ScanAssemblerNode);

    std::string robotFrame;
    node->declare_parameter<std::string>("robot_frame", "base_link");
    node->get_parameter("robot_frame", robotFrame);

    std::shared_ptr<PM::Transformation> transformator = PM::get().TransformationRegistrar.create("RigidTransformation");

    tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(node->get_clock()));
    tf2_ros::TransformListener tfListener(*tfBuffer);

    newFirstCloud.store(false);
    newSecondCloud.store(false);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPublisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", 1);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr firstCloudSubscription =
            node->create_subscription<sensor_msgs::msg::PointCloud2>("cloud_in_1", 1,
                                                                     std::bind(&ScanAssemblerNode::firstCloudCallback, node, std::placeholders::_1));
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr secondCloudSubscription =
            node->create_subscription<sensor_msgs::msg::PointCloud2>("cloud_in_2", 1,
                                                                     std::bind(&ScanAssemblerNode::secondCloudCallback, node, std::placeholders::_1));

    rclcpp::Rate loopRate(20);
    while(rclcpp::ok())
    {
        if(newFirstCloud.load() && newSecondCloud.load())
        {
            newFirstCloud.store(false);
            newSecondCloud.store(false);
            rclcpp::Time earliestStamp;
            if(rclcpp::Time(firstCloud.header.stamp) < rclcpp::Time(secondCloud.header.stamp))
            {
                earliestStamp = firstCloud.header.stamp;
            }
            else
            {
                earliestStamp = secondCloud.header.stamp;
            }
            try
            {
                PM::TransformationParameters firstTransform = findTransform(firstCloud.header.frame_id, robotFrame,
                                                                            firstCloud.header.stamp, 4);
                PM::TransformationParameters secondTransform = findTransform(secondCloud.header.frame_id, robotFrame,
                                                                             secondCloud.header.stamp, 4);
                PM::DataPoints firstCloudPM = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(firstCloud);
                PM::DataPoints secondCloudPM = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(secondCloud);
                PM::DataPoints assembledCloudPM = transformator->compute(firstCloudPM, firstTransform);
                assembledCloudPM.concatenate(transformator->compute(secondCloudPM, secondTransform));
                sensor_msgs::msg::PointCloud2 assembledCloud = PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(assembledCloudPM, robotFrame, earliestStamp);
                cloudPublisher->publish(assembledCloud);
            }
            catch(const tf2::TransformException& ex)
            {
                RCLCPP_WARN(node->get_logger(), "%s", ex.what());
            }
        }

        rclcpp::spin_some(node);
        loopRate.sleep();
    }

    return 0;
}
