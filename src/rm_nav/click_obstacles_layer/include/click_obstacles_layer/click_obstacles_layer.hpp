#pragma once

#include <geometry_msgs/msg/point_stamped.hpp>
#include <mutex>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>

namespace click_obstacles_layer {

  struct ObstaclePoint {
    double x;
    double y;
  };

  class ClickObstaclesLayer : public nav2_costmap_2d::Layer {
  public:
    ClickObstaclesLayer();
    ~ClickObstaclesLayer() override = default;

    void onInitialize() override;
    void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                      double* min_y, double* max_x, double* max_y) override;
    void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                     int max_j) override;
    void reset() override;

    bool isClearable() override { return true; }

    void activate() override;
    void deactivate() override;

  private:
    void onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void onClear(const std_msgs::msg::Empty::SharedPtr);
    void onSetRadius(const std_msgs::msg::Float32::SharedPtr msg);
    void onEnable(const std_msgs::msg::Bool::SharedPtr msg);

    double obstacle_radius_{0.20};
    bool   enabled_{true};

    std::mutex                 mutex_;
    std::vector<ObstaclePoint> obstacles_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_clicked_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr             sub_clear_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           sub_radius_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              sub_enable_;
  };

} // namespace click_obstacles_layer