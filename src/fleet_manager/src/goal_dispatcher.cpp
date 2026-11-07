// Sends each goal to the closest robot. A goal arrives on /goal_pose (RViz "2D Goal Pose"). Every robot's
// nav2 planner is asked for a path from where that robot is now (/robotN/compute_path_to_pose). The robot
// with the shortest path gets the goal (/robotN/navigate_to_pose). "Closest" is path length, not straight
// line, because a robot just behind a wall can have a long route.
// Idle robots are preferred; if every robot that has a path is busy, the closest one is re-tasked.
// ponytail: no timeout on planning; a robot whose planner never answers blocks that goal. Add one if needed.
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using ComputePath = nav2_msgs::action::ComputePathToPose;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class GoalDispatcher : public rclcpp::Node {
public:
    GoalDispatcher() : Node("goal_dispatcher") {
        robots_ = declare_parameter("robots", std::vector<std::string>{"robot1", "robot2", "robot3"});
        const size_t n = robots_.size();
        length_.resize(n);
        busy_.assign(n, false);
        nav_gen_.assign(n, 0);
        for (const auto& r : robots_) {
            plan_clients_.push_back(rclcpp_action::create_client<ComputePath>(this, "/" + r + "/compute_path_to_pose"));
            nav_clients_.push_back(rclcpp_action::create_client<NavigateToPose>(this, "/" + r + "/navigate_to_pose"));
        }
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&GoalDispatcher::on_goal, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "goal_dispatcher up: %zu robots, waiting for /goal_pose", n);
    }

private:
    void on_goal(geometry_msgs::msg::PoseStamped::ConstSharedPtr goal) {
        goal_ = *goal;
        const int gen = ++gen_;  // a newer goal makes answers for older ones irrelevant
        pending_ = 0;
        for (size_t i = 0; i < robots_.size(); ++i) {
            length_[i] = std::numeric_limits<double>::infinity();
            if (!plan_clients_[i]->action_server_is_ready()) {
                RCLCPP_WARN(get_logger(), "%s: planner not available, skipped", robots_[i].c_str());
                continue;
            }
            ComputePath::Goal req;
            req.goal = goal_;
            req.use_start = false;  // plan from the robot's current pose
            rclcpp_action::Client<ComputePath>::SendGoalOptions opt;
            opt.result_callback = [this, i, gen](const rclcpp_action::ClientGoalHandle<ComputePath>::WrappedResult& res) {
                if (gen != gen_) return;
                if (res.code == rclcpp_action::ResultCode::SUCCEEDED && !res.result->path.poses.empty())
                    length_[i] = path_length(res.result->path.poses);
                if (--pending_ == 0) dispatch();
            };
            opt.goal_response_callback = [this, i, gen](rclcpp_action::ClientGoalHandle<ComputePath>::SharedPtr h) {
                if (h || gen != gen_) return;  // accepted, result will follow
                RCLCPP_WARN(get_logger(), "%s: planner rejected the request", robots_[i].c_str());
                if (--pending_ == 0) dispatch();
            };
            ++pending_;
            plan_clients_[i]->async_send_goal(req, opt);
        }
        RCLCPP_INFO(get_logger(), "goal (%.2f, %.2f): asking %d planners", goal_.pose.position.x,
                    goal_.pose.position.y, pending_);
        if (pending_ == 0) RCLCPP_ERROR(get_logger(), "no planner available, goal dropped");
    }

    static double path_length(const std::vector<geometry_msgs::msg::PoseStamped>& p) {
        double len = 0;
        for (size_t k = 1; k < p.size(); ++k)
            len += std::hypot(p[k].pose.position.x - p[k - 1].pose.position.x,
                              p[k].pose.position.y - p[k - 1].pose.position.y);
        return len;
    }

    void dispatch() {
        int best = -1;
        for (const bool idle_only : {true, false}) {
            for (size_t i = 0; i < robots_.size(); ++i)
                if (std::isfinite(length_[i]) && (!idle_only || !busy_[i]) && (best < 0 || length_[i] < length_[best]))
                    best = static_cast<int>(i);
            if (best >= 0) break;
        }
        std::string table;
        for (size_t i = 0; i < robots_.size(); ++i)
            table += " " + robots_[i] + "=" + (std::isfinite(length_[i]) ? std::to_string(length_[i]).substr(0, 5) + "m" : "no path") +
                     (busy_[i] ? "(busy)" : "");
        if (best < 0) {
            RCLCPP_ERROR(get_logger(), "no robot can reach the goal:%s", table.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "path lengths:%s -> %s", table.c_str(), robots_[best].c_str());

        NavigateToPose::Goal req;
        req.pose = goal_;
        const int ng = ++nav_gen_[best];  // re-tasking a busy robot: ignore its old goal's result
        busy_[best] = true;
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions opt;
        opt.result_callback = [this, best, ng](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& res) {
            if (ng != nav_gen_[best]) return;
            busy_[best] = false;
            const char* r = res.code == rclcpp_action::ResultCode::SUCCEEDED ? "reached the goal"
                          : res.code == rclcpp_action::ResultCode::CANCELED  ? "was canceled" : "failed";
            RCLCPP_INFO(get_logger(), "%s %s", robots_[best].c_str(), r);
        };
        nav_clients_[best]->async_send_goal(req, opt);
    }

    std::vector<std::string> robots_;
    geometry_msgs::msg::PoseStamped goal_;
    int gen_ = 0, pending_ = 0;
    std::vector<double> length_;
    std::vector<bool> busy_;
    std::vector<int> nav_gen_;
    std::vector<rclcpp_action::Client<ComputePath>::SharedPtr> plan_clients_;
    std::vector<rclcpp_action::Client<NavigateToPose>::SharedPtr> nav_clients_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalDispatcher>());
    rclcpp::shutdown();
    return 0;
}
