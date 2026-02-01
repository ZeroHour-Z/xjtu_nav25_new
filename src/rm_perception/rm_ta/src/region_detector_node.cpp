/**
 * @file region_detector_node.cpp
 * @brief 区域检测节点 - 检测机器人是否在特殊区域（如颠簸路段）
 * 
 * 功能：
 * 1. 定义多个特殊区域（多边形）
 * 2. 订阅路径和TF，检测机器人是否即将进入特殊区域
 * 3. 发布区域状态和期望航向角
 * 4. 在RViz中可视化区域
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <set>

// 区域类型定义（与 rm_comm_ros2/protocol.h 保持一致）
enum RegionType {
  REGION_LOCAL = 0,
  REGION_FLAT,
  REGION_HOLE,
  REGION_SLOPE,
  REGION_STEP,
  REGION_FLUCTUATE,
};

// 区域配置结构
struct RegionConfig {
  std::string name;
  RegionType type;
  std::vector<std::pair<double, double>> polygon;
  double yaw_desired;  // 进入该区域时的期望航向角
};

class RegionDetectorNode : public rclcpp::Node {
public:
  RegionDetectorNode() 
    : Node("region_detector_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) 
  {
    // 声明参数
    this->declare_parameter<double>("lookahead_distance", 0.5);
    this->declare_parameter<double>("publish_rate", 20.0);
    
    // 颠簸区域参数（可以定义多个区域）
    // 格式：[x1, y1, x2, y2, x3, y3, ...]
    this->declare_parameter<std::vector<double>>("fluctuate_region_1", 
        std::vector<double>{});
    this->declare_parameter<double>("fluctuate_region_1_yaw", 0.0);
    
    this->declare_parameter<std::vector<double>>("fluctuate_region_2", 
        std::vector<double>{});
    this->declare_parameter<double>("fluctuate_region_2_yaw", 0.0);
    
    this->declare_parameter<std::vector<double>>("fluctuate_region_3", 
        std::vector<double>{});
    this->declare_parameter<double>("fluctuate_region_3_yaw", 0.0);
    
    // 加载区域配置
    loadRegions();
    
    // 发布器
    region_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/region_type", 10);
    yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>("/region_yaw_desired", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/region_markers", 10);
    
    // 订阅路径
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10, 
        std::bind(&RegionDetectorNode::onPath, this, std::placeholders::_1));
    
    // 定时器
    double rate = 20.0;
    this->get_parameter("publish_rate", rate);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
        std::bind(&RegionDetectorNode::onTimer, this));
    
    marker_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&RegionDetectorNode::publishMarkers, this));
    
    RCLCPP_INFO(this->get_logger(), "Region detector started with %zu regions", 
                regions_.size());
  }

private:
  // 加载区域配置
  void loadRegions() {
    regions_.clear();
    
    // 加载颠簸区域1
    loadSingleRegion("fluctuate_region_1", "fluctuate_region_1_yaw", 
                     "Fluctuate1", REGION_FLUCTUATE);
    // 加载颠簸区域2
    loadSingleRegion("fluctuate_region_2", "fluctuate_region_2_yaw", 
                     "Fluctuate2", REGION_FLUCTUATE);
    // 加载颠簸区域3
    loadSingleRegion("fluctuate_region_3", "fluctuate_region_3_yaw", 
                     "Fluctuate3", REGION_FLUCTUATE);
  }
  
  void loadSingleRegion(const std::string& polygon_param, 
                        const std::string& yaw_param,
                        const std::string& name,
                        RegionType type) {
    std::vector<double> polygon_data;
    this->get_parameter(polygon_param, polygon_data);
    
    if (polygon_data.size() >= 6 && polygon_data.size() % 2 == 0) {
      RegionConfig region;
      region.name = name;
      region.type = type;
      this->get_parameter(yaw_param, region.yaw_desired);
      
      for (size_t i = 0; i < polygon_data.size(); i += 2) {
        region.polygon.push_back({polygon_data[i], polygon_data[i + 1]});
      }
      
      regions_.push_back(region);
      RCLCPP_INFO(this->get_logger(), "Loaded region '%s' with %zu vertices, yaw=%.2f",
                  name.c_str(), region.polygon.size(), region.yaw_desired);
    }
  }
  
  // 判断点是否在多边形内 (射线法)
  bool isPointInPolygon(double x, double y, 
                        const std::vector<std::pair<double, double>>& polygon) {
    if (polygon.size() < 3) return false;
    
    bool inside = false;
    size_t n = polygon.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
      double xi = polygon[i].first, yi = polygon[i].second;
      double xj = polygon[j].first, yj = polygon[j].second;
      
      if (((yi > y) != (yj > y)) &&
          (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
        inside = !inside;
      }
    }
    return inside;
  }
  
  // 计算点到多边形边界的最近距离
  double distanceToPolygon(double x, double y, 
                           const std::vector<std::pair<double, double>>& polygon) {
    if (polygon.size() < 3) return std::numeric_limits<double>::max();
    
    double min_dist = std::numeric_limits<double>::max();
    size_t n = polygon.size();
    
    for (size_t i = 0; i < n; ++i) {
      size_t j = (i + 1) % n;
      double x1 = polygon[i].first, y1 = polygon[i].second;
      double x2 = polygon[j].first, y2 = polygon[j].second;
      
      double dx = x2 - x1;
      double dy = y2 - y1;
      double t = std::max(0.0, std::min(1.0, 
          ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy + 1e-9)));
      double closest_x = x1 + t * dx;
      double closest_y = y1 + t * dy;
      double dist = std::hypot(x - closest_x, y - closest_y);
      min_dist = std::min(min_dist, dist);
    }
    return min_dist;
  }
  
  // 路径回调
  void onPath(const nav_msgs::msg::Path::SharedPtr msg) {
    last_path_ = *msg;
    has_path_ = true;
    
    // 检查路径经过哪些区域
    path_regions_.clear();
    for (size_t i = 0; i < regions_.size(); ++i) {
      for (const auto& pose : msg->poses) {
        if (isPointInPolygon(pose.pose.position.x, pose.pose.position.y, 
                             regions_[i].polygon)) {
          path_regions_.insert(i);
          break;
        }
      }
    }
    
    if (!path_regions_.empty()) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Path goes through %zu special region(s)", path_regions_.size());
    }
  }
  
  // 获取当前机器人在 map 坐标系的位置
  bool getRobotPose(double& x, double& y, double& yaw) {
    try {
      auto tf = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      
      tf2::Quaternion q;
      q.setX(tf.transform.rotation.x);
      q.setY(tf.transform.rotation.y);
      q.setZ(tf.transform.rotation.z);
      q.setW(tf.transform.rotation.w);
      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      return true;
    } catch (const tf2::TransformException& ex) {
      return false;
    }
  }
  
  // 定时器回调
  void onTimer() {
    double robot_x, robot_y, robot_yaw;
    if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
      return;
    }
    
    double lookahead_dist = 0.5;
    this->get_parameter("lookahead_distance", lookahead_dist);
    
    // 检测当前区域
    RegionType current_region = REGION_FLAT;
    double yaw_desired = 0.0;
    bool in_special_region = false;
    
    for (size_t i = 0; i < regions_.size(); ++i) {
      const auto& region = regions_[i];
      
      // 检查是否在区域内
      if (isPointInPolygon(robot_x, robot_y, region.polygon)) {
        current_region = region.type;
        yaw_desired = region.yaw_desired;
        in_special_region = true;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "Robot IN region '%s'! pos=(%.2f, %.2f)", 
                             region.name.c_str(), robot_x, robot_y);
        break;
      }
      
      // 检查是否即将进入区域（路径经过且距离近）
      if (path_regions_.count(i) > 0) {
        double dist = distanceToPolygon(robot_x, robot_y, region.polygon);
        if (dist < lookahead_dist) {
          current_region = region.type;
          yaw_desired = region.yaw_desired;
          in_special_region = true;
          
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                               "Robot APPROACHING region '%s'! dist=%.2f", 
                               region.name.c_str(), dist);
          break;
        }
      }
    }
    
    // 发布区域类型
    std_msgs::msg::UInt8 region_msg;
    region_msg.data = static_cast<uint8_t>(current_region);
    region_pub_->publish(region_msg);
    
    // 发布期望航向角
    std_msgs::msg::Float32 yaw_msg;
    yaw_msg.data = static_cast<float>(yaw_desired);
    yaw_pub_->publish(yaw_msg);
    
    // 更新状态用于可视化
    current_region_ = current_region;
    in_special_region_ = in_special_region;
  }
  
  // 发布可视化 Marker
  void publishMarkers() {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    
    for (size_t i = 0; i < regions_.size(); ++i) {
      const auto& region = regions_[i];
      
      // 边界线
      visualization_msgs::msg::Marker line_marker;
      line_marker.header.frame_id = "map";
      line_marker.header.stamp = this->now();
      line_marker.ns = "region_boundary";
      line_marker.id = id++;
      line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_marker.action = visualization_msgs::msg::Marker::ADD;
      line_marker.scale.x = 0.05;
      
      // 根据状态设置颜色
      bool is_active = (in_special_region_ && path_regions_.count(i) > 0);
      if (is_active) {
        line_marker.color.r = 1.0f;
        line_marker.color.g = 0.0f;
        line_marker.color.b = 0.0f;
        line_marker.color.a = 1.0f;
      } else if (path_regions_.count(i) > 0) {
        line_marker.color.r = 1.0f;
        line_marker.color.g = 0.5f;
        line_marker.color.b = 0.0f;
        line_marker.color.a = 0.8f;
      } else {
        line_marker.color.r = 1.0f;
        line_marker.color.g = 1.0f;
        line_marker.color.b = 0.0f;
        line_marker.color.a = 0.6f;
      }
      
      for (const auto& pt : region.polygon) {
        geometry_msgs::msg::Point p;
        p.x = pt.first;
        p.y = pt.second;
        p.z = 0.05;
        line_marker.points.push_back(p);
      }
      // 闭合
      if (!region.polygon.empty()) {
        geometry_msgs::msg::Point p;
        p.x = region.polygon.front().first;
        p.y = region.polygon.front().second;
        p.z = 0.05;
        line_marker.points.push_back(p);
      }
      
      line_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
      marker_array.markers.push_back(line_marker);
      
      // 填充区域
      visualization_msgs::msg::Marker fill_marker;
      fill_marker.header.frame_id = "map";
      fill_marker.header.stamp = this->now();
      fill_marker.ns = "region_fill";
      fill_marker.id = id++;
      fill_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      fill_marker.action = visualization_msgs::msg::Marker::ADD;
      fill_marker.scale.x = 1.0;
      fill_marker.scale.y = 1.0;
      fill_marker.scale.z = 1.0;
      
      if (is_active) {
        fill_marker.color.r = 1.0f;
        fill_marker.color.g = 0.0f;
        fill_marker.color.b = 0.0f;
        fill_marker.color.a = 0.3f;
      } else {
        fill_marker.color.r = 1.0f;
        fill_marker.color.g = 1.0f;
        fill_marker.color.b = 0.0f;
        fill_marker.color.a = 0.15f;
      }
      
      // 扇形分解多边形
      if (region.polygon.size() >= 3) {
        for (size_t j = 1; j < region.polygon.size() - 1; ++j) {
          geometry_msgs::msg::Point p0, p1, p2;
          p0.x = region.polygon[0].first;
          p0.y = region.polygon[0].second;
          p0.z = 0.02;
          
          p1.x = region.polygon[j].first;
          p1.y = region.polygon[j].second;
          p1.z = 0.02;
          
          p2.x = region.polygon[j + 1].first;
          p2.y = region.polygon[j + 1].second;
          p2.z = 0.02;
          
          fill_marker.points.push_back(p0);
          fill_marker.points.push_back(p1);
          fill_marker.points.push_back(p2);
        }
      }
      
      fill_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
      marker_array.markers.push_back(fill_marker);
      
      // 区域名称标签
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = "map";
      text_marker.header.stamp = this->now();
      text_marker.ns = "region_label";
      text_marker.id = id++;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      
      // 计算多边形中心
      double cx = 0, cy = 0;
      for (const auto& pt : region.polygon) {
        cx += pt.first;
        cy += pt.second;
      }
      cx /= region.polygon.size();
      cy /= region.polygon.size();
      
      text_marker.pose.position.x = cx;
      text_marker.pose.position.y = cy;
      text_marker.pose.position.z = 0.5;
      text_marker.scale.z = 0.3;
      text_marker.color.r = 1.0f;
      text_marker.color.g = 1.0f;
      text_marker.color.b = 1.0f;
      text_marker.color.a = 1.0f;
      text_marker.text = region.name;
      text_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
      marker_array.markers.push_back(text_marker);
    }
    
    marker_pub_->publish(marker_array);
  }

  // 成员变量
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr region_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  
  std::vector<RegionConfig> regions_;
  nav_msgs::msg::Path last_path_;
  bool has_path_{false};
  std::set<size_t> path_regions_;  // 路径经过的区域索引
  
  RegionType current_region_{REGION_FLAT};
  bool in_special_region_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RegionDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
