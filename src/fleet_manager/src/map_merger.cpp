// Merges every robot's SLAM map (/robotN/map, frame robotN/map) into one /map in the `world` frame.
// Where each map sits comes from TF (world -> robotN/map, published by the launch file at the spawn
// pose). Every output cell samples each robot's map: occupied if any robot says occupied, else free
// if any says free, else unknown.
// ponytail: trusts the spawn poses exactly (SLAM's map origin = spawn pose). If walls come out doubled,
// estimate world -> robotN/map from scan matching instead (e.g. multirobot_map_merge).
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class MapMerger : public rclcpp::Node {
public:
    MapMerger() : Node("map_merger") {
        robots_ = declare_parameter("robots", std::vector<std::string>{"robot1", "robot2", "robot3"});
        world_ = declare_parameter<std::string>("world_frame", "world");
        res_ = declare_parameter("resolution", 0.05);
        maps_.resize(robots_.size());
        for (size_t i = 0; i < robots_.size(); ++i)
            subs_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/" + robots_[i] + "/map", rclcpp::QoS(1).reliable(),
                [this, i](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { maps_[i] = m; }));
        // transient_local: RViz / explorers started later still get the last map
        pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).reliable().transient_local());
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&MapMerger::merge, this));
        RCLCPP_INFO(get_logger(), "map_merger up: %zu robots -> /map (%s)", robots_.size(), world_.c_str());
    }

private:
    struct Placed { nav_msgs::msg::OccupancyGrid::ConstSharedPtr m; double tx, ty, c, s; };  // world <- map

    void merge() {
        std::vector<Placed> placed;
        double x0 = std::numeric_limits<double>::infinity(), y0 = x0, x1 = -x0, y1 = -x0;
        for (const auto& m : maps_) {
            if (!m) continue;
            geometry_msgs::msg::TransformStamped tf;
            try {
                tf = tf_buffer_->lookupTransform(world_, m->header.frame_id, tf2::TimePointZero);
            } catch (const tf2::TransformException& e) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", e.what());
                continue;
            }
            const auto& q = tf.transform.rotation;
            const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
            const Placed p{m, tf.transform.translation.x, tf.transform.translation.y, std::cos(yaw), std::sin(yaw)};
            placed.push_back(p);
            // world bounding box of this map's four corners
            const auto& o = m->info.origin.position;
            const double w = m->info.width * m->info.resolution, h = m->info.height * m->info.resolution;
            for (const double mx : {o.x, o.x + w})
                for (const double my : {o.y, o.y + h}) {
                    const double wx = p.tx + p.c * mx - p.s * my, wy = p.ty + p.s * mx + p.c * my;
                    x0 = std::min(x0, wx); x1 = std::max(x1, wx);
                    y0 = std::min(y0, wy); y1 = std::max(y1, wy);
                }
        }
        if (placed.empty()) return;

        nav_msgs::msg::OccupancyGrid out;
        out.header.frame_id = world_;
        out.header.stamp = now();
        out.info.resolution = res_;
        out.info.origin.position.x = std::floor(x0 / res_) * res_;
        out.info.origin.position.y = std::floor(y0 / res_) * res_;
        out.info.origin.orientation.w = 1.0;
        out.info.width = static_cast<unsigned>(std::ceil((x1 - out.info.origin.position.x) / res_));
        out.info.height = static_cast<unsigned>(std::ceil((y1 - out.info.origin.position.y) / res_));
        out.data.assign(out.info.width * out.info.height, -1);

        for (unsigned row = 0; row < out.info.height; ++row)
            for (unsigned col = 0; col < out.info.width; ++col) {
                const double wx = out.info.origin.position.x + (col + 0.5) * res_;
                const double wy = out.info.origin.position.y + (row + 0.5) * res_;
                int8_t v = -1;
                for (const auto& p : placed) {
                    const auto& i = p.m->info;
                    const double dx = wx - p.tx, dy = wy - p.ty;  // world -> robot map frame
                    const double mx = p.c * dx + p.s * dy, my = -p.s * dx + p.c * dy;
                    const int cx = static_cast<int>(std::floor((mx - i.origin.position.x) / i.resolution));
                    const int cy = static_cast<int>(std::floor((my - i.origin.position.y) / i.resolution));
                    if (cx < 0 || cy < 0 || cx >= static_cast<int>(i.width) || cy >= static_cast<int>(i.height)) continue;
                    const int8_t m = p.m->data[cy * i.width + cx];
                    if (m >= 50) { v = 100; break; }  // occupied by anyone wins
                    if (m >= 0) v = 0;
                }
                out.data[row * out.info.width + col] = v;
            }
        pub_->publish(out);
    }

    std::vector<std::string> robots_;
    std::string world_;
    double res_;
    std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> maps_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> subs_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMerger>());
    rclcpp::shutdown();
    return 0;
}
