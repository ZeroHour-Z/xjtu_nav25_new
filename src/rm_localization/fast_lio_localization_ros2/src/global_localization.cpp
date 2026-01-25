#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>
#include <cmath>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

class GlobalLocalizationNode : public rclcpp::Node {
public:
	GlobalLocalizationNode() : rclcpp::Node("fast_lio_localization") {
		// Parameters
		this->declare_parameter<bool>("map2odom_completed", false);
		this->declare_parameter<int>("region", 0);
		this->declare_parameter<double>("freq_localization", 0.5);
		this->declare_parameter<double>("localization_th", 0.1); // MSE threshold (lower better)
		this->declare_parameter<double>("map_voxel_size", 0.2);
		this->declare_parameter<double>("scan_voxel_size", 0.1);
		this->declare_parameter<double>("fov", 6.28);
		this->declare_parameter<double>("fov_far", 30.0);
		this->declare_parameter<std::string>("map_frame", "map3d");
		this->declare_parameter<std::string>("odom_frame", "camera_init");
		this->declare_parameter<std::string>("base_link_frame", "base_link");
		// Keep param for compatibility; not used
		this->declare_parameter<bool>("use_gicp", false);

		// QoS
		rclcpp::QoS qos_reliable(10);
		qos_reliable.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
		rclcpp::QoS qos_transient(1);
		qos_transient.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
		qos_transient.transient_local();

		// Publishers
		pub_pc_in_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cur_scan_in_map", 1);
		pub_submap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/submap", 1);
		pub_map_to_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/map_to_odom", 1);
		pub_initialpose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", qos_transient);

		// Subscriptions
		sub_cloud_registered_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_registered", 1, std::bind(&GlobalLocalizationNode::cbSaveCurScan, this, std::placeholders::_1));
		sub_aft_mapped_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&GlobalLocalizationNode::cbSaveCurOdom, this, std::placeholders::_1));
		sub_map3d_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/map3d", qos_reliable, std::bind(&GlobalLocalizationNode::cbInitGlobalMap, this, std::placeholders::_1));
		// Subscribe to /initialpose from RViz "2D Pose Estimate"
		sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"/initialpose_rviz", 10,
			std::bind(&GlobalLocalizationNode::cbInitialPose, this, std::placeholders::_1));

		// Thread
		worker_ = std::thread(&GlobalLocalizationNode::initializeAndRun, this);
		RCLCPP_INFO(this->get_logger(), "Localization Node Inited (C++) ...");
	}

	~GlobalLocalizationNode() override {
		alive_.store(false);
		if (worker_.joinable()) worker_.join();
	}

private:
	// Utils
	static Eigen::Matrix4d poseToMat(const nav_msgs::msg::Odometry &odom) {
		const auto &p = odom.pose.pose.position;
		const auto &q = odom.pose.pose.orientation;
		Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
		Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
		T.block<3,3>(0,0) = quat.normalized().toRotationMatrix();
		T(0,3) = p.x;
		T(1,3) = p.y;
		T(2,3) = p.z;
		return T;
	}

	static Eigen::Matrix4d inverseSE3(const Eigen::Matrix4d &T) {
		Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
		inv.block<3,3>(0,0) = T.block<3,3>(0,0).transpose();
		inv.block<3,1>(0,3) = -inv.block<3,3>(0,0) * T.block<3,1>(0,3);
		return inv;
	}

	static pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDownSample(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, double voxel) {
		pcl::VoxelGrid<pcl::PointXYZ> vg;
		vg.setLeafSize(voxel, voxel, voxel);
		vg.setInputCloud(cloud);
		auto filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
		vg.filter(*filtered);
		return filtered;
	}

	pcl::PointCloud<pcl::PointNormal>::Ptr cropGlobalMapInFOV(const pcl::PointCloud<pcl::PointNormal>::ConstPtr &global_map,
			const Eigen::Matrix4d &pose_estimation,
			const nav_msgs::msg::Odometry &cur_odom) {
		Eigen::Matrix4d T_odom_to_base = poseToMat(cur_odom);
		Eigen::Matrix4d T_map_to_base = pose_estimation * T_odom_to_base;
		Eigen::Matrix4d T_base_to_map = inverseSE3(T_map_to_base);

		double FOV = this->get_parameter("fov").as_double();
		double FOV_FAR = this->get_parameter("fov_far").as_double();

		auto submap = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
		submap->points.reserve(global_map->points.size() / 10);
		for (const auto &pt_map : global_map->points) {
			Eigen::Vector4d pm(pt_map.x, pt_map.y, pt_map.z, 1.0);
			Eigen::Vector4d pb = T_base_to_map * pm;
			double x = pb.x();
			double y = pb.y();
			double z = pb.z();
			double ang = std::atan2(y, x);
			bool in = false;
			if (FOV > 3.14) {
				in = (x < FOV_FAR) && (std::fabs(ang) < FOV / 2.0);
			} else {
				in = (x > 0.0) && (x < FOV_FAR) && (std::fabs(ang) < FOV / 2.0);
			}
			if (in) {
				pcl::PointNormal p;
				p.x = static_cast<float>(pt_map.x);
				p.y = static_cast<float>(pt_map.y);
				p.z = static_cast<float>(pt_map.z);
				p.normal_x = pt_map.normal_x;
				p.normal_y = pt_map.normal_y;
				p.normal_z = pt_map.normal_z;
				submap->points.push_back(p);
			}
		}
		submap->width = static_cast<uint32_t>(submap->points.size());
		submap->height = 1; submap->is_dense = false;

		// Publish for debug (downsampled for bandwidth)
		if (!submap->empty()) {
			auto ds = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
			ds->points.reserve(submap->points.size());
			for (const auto &pn : submap->points) {
				pcl::PointXYZ p; p.x = pn.x; p.y = pn.y; p.z = pn.z; ds->points.push_back(p);
			}
			ds->width = static_cast<uint32_t>(ds->points.size()); ds->height = 1; ds->is_dense = false;
			if (ds->size() > 200000) ds->points.resize(200000);
			sensor_msgs::msg::PointCloud2 msg;
			pcl::toROSMsg(*ds, msg);
			msg.header.stamp = this->now();
			msg.header.frame_id = this->get_parameter("map_frame").as_string();
			pub_submap_->publish(msg);
		}
		return submap;
	}

	// Estimate normals and build PointNormal for scan
	static pcl::PointCloud<pcl::PointNormal>::Ptr buildScanWithNormals(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &scan_xyz, double radius) {
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(scan_xyz);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree);
		if (radius > 0.0) ne.setRadiusSearch(radius); else ne.setKSearch(20);
		auto normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
		ne.compute(*normals);
		auto out = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
		out->points.resize(scan_xyz->points.size());
		for (size_t i = 0; i < scan_xyz->points.size(); ++i) {
			pcl::PointNormal pn;
			pn.x = scan_xyz->points[i].x;
			pn.y = scan_xyz->points[i].y;
			pn.z = scan_xyz->points[i].z;
			pn.normal_x = normals->points[i].normal_x;
			pn.normal_y = normals->points[i].normal_y;
			pn.normal_z = normals->points[i].normal_z;
			out->points[i] = pn;
		}
		out->width = static_cast<uint32_t>(out->points.size()); out->height = 1; out->is_dense = false;
		return out;
	}

	static pcl::PointCloud<pcl::PointNormal>::Ptr voxelDownSampleNormals(const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud, double voxel) {
		pcl::VoxelGrid<pcl::PointNormal> vg;
		vg.setLeafSize(voxel, voxel, voxel);
		vg.setInputCloud(cloud);
		auto filtered = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
		vg.filter(*filtered);
		return filtered;
	}

	std::pair<Eigen::Matrix4d, double> registrationAtScale(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &scan,
			const pcl::PointCloud<pcl::PointNormal>::ConstPtr &map,
			const Eigen::Matrix4d &initial, double scale) {
		double scan_voxel = this->get_parameter("scan_voxel_size").as_double() * scale;
		double map_voxel = this->get_parameter("map_voxel_size").as_double() * scale;
		// Downsample scan first (XYZ), then estimate normals to reduce cost
		auto ds_scan_xyz = voxelDownSample(scan, std::max(0.01, scan_voxel));
		// Estimate normals for scan with a radius tied to voxel size
		double normal_radius = std::max(0.02, 2.5 * scan_voxel);
		auto ds_scan = buildScanWithNormals(ds_scan_xyz, normal_radius);

		 // 使用预先降采样的全局地图
		auto ds_map = global_map_ds_ ? global_map_ds_ : map;

		pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
		icp.setMaxCorrespondenceDistance(1.0 * scale);
		icp.setMaximumIterations(35);
		icp.setTransformationEpsilon(1e-8);
		icp.setEuclideanFitnessEpsilon(1e-8);
		icp.setInputSource(ds_scan);
		icp.setInputTarget(ds_map);
		pcl::PointCloud<pcl::PointNormal> aligned;
		Eigen::Matrix4f initf = initial.cast<float>();
		icp.align(aligned, initf);
		Eigen::Matrix4d T = icp.getFinalTransformation().cast<double>();
		double mse = icp.getFitnessScore(1.0 * scale); // lower is better
		return {T, mse};
	}

	void publishInitialPose() {
		geometry_msgs::msg::PoseWithCovarianceStamped msg;
		msg.header.frame_id = this->get_parameter("map_frame").as_string();
		msg.header.stamp = this->now();
		msg.pose.pose.position.x = 0.0;
		msg.pose.pose.position.y = 0.0;
		msg.pose.pose.position.z = 0.0;
		msg.pose.pose.orientation.w = 1.0;
		msg.pose.pose.orientation.x = 0.0;
		msg.pose.pose.orientation.y = 0.0;
		msg.pose.pose.orientation.z = 0.0;
		pub_initialpose_->publish(msg);
	}

	bool globalLocalization(Eigen::Matrix4d &pose_estimation) {
		auto gm = global_map_;
		if (!gm || !cur_scan_ || !cur_odom_) return false;

		RCLCPP_INFO(this->get_logger(), "Global localization by scan-to-map matching ...");
		auto submap = cropGlobalMapInFOV(gm, pose_estimation, *cur_odom_);

		auto [T1, mse1] = registrationAtScale(cur_scan_, submap, pose_estimation, 5.0);
		auto [T2, mse2] = registrationAtScale(cur_scan_, submap, T1, 1.0);
		double mse = mse2;

		bool map2odom_completed = this->get_parameter("map2odom_completed").as_bool();
		int region_id = this->get_parameter("region").as_int();
		if (map2odom_completed || (region_id == 2 || region_id == 4 || region_id == 5 || region_id == 6 || region_id == 7)) {
			RCLCPP_WARN(this->get_logger(), "LOCALIZATION DISABLED: region=%d, map2odom_completed=%s", region_id, map2odom_completed ? "true" : "false");
			return true;
		}

		// Direct MSE threshold: mse < localization_th
		double mse_th = std::max(1e-6, this->get_parameter("localization_th").as_double());
		if (mse < mse_th) {
			pose_estimation = T2;

			nav_msgs::msg::Odometry map_to_odom;
			Eigen::Quaterniond q(T2.block<3,3>(0,0)); q.normalize();
			map_to_odom.pose.pose.position.x = T2(0,3);
			map_to_odom.pose.pose.position.y = T2(1,3);
			map_to_odom.pose.pose.position.z = T2(2,3);
			map_to_odom.pose.pose.orientation.x = q.x();
			map_to_odom.pose.pose.orientation.y = q.y();
			map_to_odom.pose.pose.orientation.z = q.z();
			map_to_odom.pose.pose.orientation.w = q.w();
			map_to_odom.header.stamp = cur_odom_->header.stamp;
			map_to_odom.header.frame_id = this->get_parameter("map_frame").as_string();
			pub_map_to_odom_->publish(map_to_odom);
			RCLCPP_INFO(this->get_logger(), "relocalization mse: %.6f (mse_th=%.6f)", mse, mse_th);
			return true;
		} else {
			RCLCPP_WARN(this->get_logger(), "Not match (mse=%.6f, mse_th=%.6f)", mse, mse_th);
			return false;
		}
	}

	void initializeAndRun() {
		while (alive_.load() && rclcpp::ok() && !global_map_) {
			RCLCPP_WARN(this->get_logger(), "Waiting for global map ...");
			rclcpp::sleep_for(std::chrono::milliseconds(500));
		}

		publishInitialPose();
		while (alive_.load() && rclcpp::ok()) {
			if (!cur_scan_) {
				RCLCPP_WARN(this->get_logger(), "First scan not received!");
				rclcpp::sleep_for(std::chrono::seconds(1));
				continue;
			}
			if (!cur_odom_) {
				RCLCPP_WARN(this->get_logger(), "Waiting for odom ...");
				rclcpp::sleep_for(std::chrono::seconds(1));
				continue;
			}
			bool ok = globalLocalization(T_map_to_odom_);
			if (ok) break;
			rclcpp::sleep_for(std::chrono::seconds(1));
		}

		RCLCPP_INFO(this->get_logger(), "Initialize successfully!");
		double freq = this->get_parameter("freq_localization").as_double();
		double period = (freq > 0.0) ? (1.0 / freq) : 2.0;
		auto sleep_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::duration<double>(period)
		);
		while (alive_.load() && rclcpp::ok()) {
			globalLocalization(T_map_to_odom_);
			rclcpp::sleep_for(sleep_duration);
		}
	}

	// Callbacks
	void cbSaveCurOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
		cur_odom_ = msg;
	}

	void cbSaveCurScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::fromROSMsg(*msg, *cloud);
		cur_scan_ = cloud;

		// republish with odom frame
		sensor_msgs::msg::PointCloud2 out;
		out = *msg; // keep data as-is
		out.header.frame_id = this->get_parameter("odom_frame").as_string();
		out.header.stamp = this->now();
		pub_pc_in_map_->publish(out);
	}

	// Callback for /initialpose from RViz "2D Pose Estimate"
	void cbInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
		RCLCPP_INFO(this->get_logger(), "Received initial pose from RViz");
		const auto &p = msg->pose.pose.position;
		const auto &q = msg->pose.pose.orientation;
		
		// Convert to transformation matrix
		Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
		Eigen::Matrix4d T_init = Eigen::Matrix4d::Identity();
		T_init.block<3,3>(0,0) = quat.normalized().toRotationMatrix();
		T_init(0,3) = p.x;
		T_init(1,3) = p.y;
		T_init(2,3) = p.z;
		
		// Update map_to_odom transform
		if (cur_odom_) {
			Eigen::Matrix4d T_odom_to_base = poseToMat(*cur_odom_);
			T_map_to_odom_ = T_init * inverseSE3(T_odom_to_base);
			RCLCPP_INFO(this->get_logger(), "Updated T_map_to_odom from RViz initial pose");
			
			// Publish map_to_odom
			nav_msgs::msg::Odometry map_to_odom;
			Eigen::Quaterniond q_out(T_map_to_odom_.block<3,3>(0,0)); q_out.normalize();
			map_to_odom.pose.pose.position.x = T_map_to_odom_(0,3);
			map_to_odom.pose.pose.position.y = T_map_to_odom_(1,3);
			map_to_odom.pose.pose.position.z = T_map_to_odom_(2,3);
			map_to_odom.pose.pose.orientation.x = q_out.x();
			map_to_odom.pose.pose.orientation.y = q_out.y();
			map_to_odom.pose.pose.orientation.z = q_out.z();
			map_to_odom.pose.pose.orientation.w = q_out.w();
			map_to_odom.header.stamp = this->now();
			map_to_odom.header.frame_id = this->get_parameter("map_frame").as_string();
			pub_map_to_odom_->publish(map_to_odom);
			
			publishInitialPose();
		} else {
			// If no odom available, use identity for odom_to_base
			T_map_to_odom_ = T_init;
			RCLCPP_WARN(this->get_logger(), "No odom available, using initial pose directly as T_map_to_odom");
		}
	}

	void cbInitGlobalMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
		if (global_map_) return;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::fromROSMsg(*msg, *cloud);
		double voxel = this->get_parameter("map_voxel_size").as_double();
		// Downsample first for efficiency, then estimate normals once (static map)
		auto ds_map_xyz = voxelDownSample(cloud, std::max(0.01, voxel));
		// Estimate normals with radius based on voxel size
		double normal_radius = std::max(0.02, 2.5 * voxel);
		global_map_ = buildScanWithNormals(ds_map_xyz, normal_radius);
		// 预先降采样带法线的全局地图
		global_map_ds_ = voxelDownSampleNormals(global_map_, std::max(0.01, voxel));
		RCLCPP_INFO(this->get_logger(), "Global map received. points=%zu (downsampled + normals)", global_map_->points.size());
	}

	// Members
	std::atomic<bool> alive_{true};
	std::thread worker_;
	pcl::PointCloud<pcl::PointNormal>::Ptr global_map_;
	pcl::PointCloud<pcl::PointNormal>::Ptr global_map_ds_; // 新增：缓存降采样后的全局地图
	pcl::PointCloud<pcl::PointXYZ>::Ptr cur_scan_;
	nav_msgs::msg::Odometry::SharedPtr cur_odom_;

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_in_map_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_map_to_odom_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initialpose_;

	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_registered_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_aft_mapped_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_map3d_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;

	Eigen::Matrix4d T_map_to_odom_ = Eigen::Matrix4d::Identity();
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<GlobalLocalizationNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
