#include <cmath>
#include <thread>

#include <ros/ros.h>

#define SV_PACKAGE_NAME "smpl_test"
#include <smpl/debug/visualize.h>
#include <smpl/debug/visualizer_ros.h>

auto MakeCubeMarker(double t) -> sbpl::visual::Marker
{
    ROS_INFO("make cube marker");
    auto marker = sbpl::visual::Marker{ };

    marker.pose.position = Eigen::Vector3d(std::sin(t), 0.0, 0.0);
    marker.pose.orientation = Eigen::Quaterniond::Identity();
    marker.shape = sbpl::visual::Cube{ 1.0, 1.0, 1.0 };
    marker.color = sbpl::visual::Color{ 1.0f, 0.0f, 0.0f, 1.0f };
    marker.frame_id = "world";
    marker.ns = "cube";

    return marker;
}

auto MakeSphereMarker(double t) -> sbpl::visual::Marker
{
    ROS_INFO("make sphere marker");
    auto marker = sbpl::visual::Marker{ };

    marker.pose.position = Eigen::Vector3d(0.0, std::sin(t), 0.0);
    marker.pose.orientation = Eigen::Quaterniond::Identity();
    marker.shape = sbpl::visual::Sphere{ 0.5 };
    marker.color = sbpl::visual::Color{ 0.0f, 1.0f, 0.0f, 1.0f };
    marker.frame_id = "world";
    marker.ns = "sphere";

    return marker;
}

void VisualizeCube()
{
    auto beginning = ros::Time::now();

    ros::Rate loop_rate(30.0);
    while (ros::ok()) {
        auto now = ros::Time::now();

        SV_SHOW_INFO_NAMED("cube", MakeCubeMarker((now - beginning).toSec()));

        loop_rate.sleep();
    }
}

void VisualizeSphere()
{
    auto beginning = ros::Time::now();

    ros::Rate loop_rate(30.0);
    while (ros::ok()) {
        auto now = ros::Time::now();

        auto make_sphere_marker = [&]() -> visualization_msgs::MarkerArray {
        };

        SV_SHOW_INFO_NAMED("sphere", MakeSphereMarker((now - beginning).toSec()));

        loop_rate.sleep();
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "debug_vis_demo");
    ros::NodeHandle nh;

    sbpl::VisualizerROS visualizer;
    sbpl::visual::set_visualizer(&visualizer);

    std::thread cube_vis_thread(VisualizeCube);
    std::thread sphere_vis_thread(VisualizeSphere);

    ROS_INFO("SV_PACKAGE_NAME: %s", SV_NAME_PREFIX);

    ros::Rate loop_rate(1.0);
    bool enabled = true;
    while (ros::ok()) {
        if (enabled) {
            sbpl::visual::set_visualization_level(
                    std::string(SV_NAME_PREFIX) + ".cube",
                    sbpl::visual::Level::Warn);
            sbpl::visual::set_visualization_level(
                    std::string(SV_NAME_PREFIX) + ".sphere",
                    sbpl::visual::Level::Info);
        } else {
            sbpl::visual::set_visualization_level(
                    std::string(SV_NAME_PREFIX) + ".cube",
                    sbpl::visual::Level::Info);
            sbpl::visual::set_visualization_level(
                    std::string(SV_NAME_PREFIX) + ".sphere",
                    sbpl::visual::Level::Warn);
        }
        enabled = !enabled;

        loop_rate.sleep();
    }

    cube_vis_thread.join();
    sphere_vis_thread.join();

    sbpl::visual::unset_visualizer();
    return 0;
}
