#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>

namespace rviz_click_obstacles {

  class ClickObstaclesPanel : public rviz_common::Panel {
    Q_OBJECT
  public:
    ClickObstaclesPanel(QWidget* parent = nullptr) : rviz_common::Panel(parent) {
      auto* layout = new QVBoxLayout();
      auto* h      = new QHBoxLayout();
      auto* label  = new QLabel("Obstacle radius (m):");
      radius_spin_ = new QDoubleSpinBox();
      radius_spin_->setRange(0.01, 5.0);
      radius_spin_->setSingleStep(0.05);
      radius_spin_->setValue(0.20);
      h->addWidget(label);
      h->addWidget(radius_spin_);

      auto* btn_apply = new QPushButton("Apply Radius");
      auto* btn_clear = new QPushButton("Clear Obstacles");
      enable_check_   = new QCheckBox("Enable Click Obstacles");
      enable_check_->setChecked(true);

      layout->addLayout(h);
      layout->addWidget(btn_apply);
      layout->addWidget(btn_clear);
      layout->addWidget(enable_check_);
      setLayout(layout);

      // Node setup
      rclcpp::NodeOptions no;
      node_ = std::make_shared<rclcpp::Node>("rviz_click_obstacles_panel", no);
      radius_pub_ =
          node_->create_publisher<std_msgs::msg::Float32>("/rviz_click_obstacles/set_radius", 10);
      clear_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/rviz_click_obstacles/clear", 10);
      enable_pub_ =
          node_->create_publisher<std_msgs::msg::Bool>("/rviz_click_obstacles/enable", 10);

      connect(btn_apply, &QPushButton::clicked, this, &ClickObstaclesPanel::onApplyRadius);
      connect(btn_clear, &QPushButton::clicked, this, &ClickObstaclesPanel::onClear);
      connect(enable_check_, &QCheckBox::toggled, this, &ClickObstaclesPanel::onEnableToggled);
    }

  private Q_SLOTS:
    void onApplyRadius() {
      std_msgs::msg::Float32 msg;
      msg.data = static_cast<float>(radius_spin_->value());
      radius_pub_->publish(msg);
    }
    void onClear() {
      std_msgs::msg::Empty msg;
      clear_pub_->publish(msg);
    }
    void onEnableToggled(bool checked) {
      std_msgs::msg::Bool msg;
      msg.data = checked;
      enable_pub_->publish(msg);
    }

  private:
    QDoubleSpinBox*                                      radius_spin_{nullptr};
    QCheckBox*                                           enable_check_{nullptr};
    rclcpp::Node::SharedPtr                              node_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr radius_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr   clear_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    enable_pub_;
  };

} // namespace rviz_click_obstacles

PLUGINLIB_EXPORT_CLASS(rviz_click_obstacles::ClickObstaclesPanel, rviz_common::Panel)

#include "click_obstacles_panel.moc"