#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <mot/MOT.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pedsim_msgs/TrackedPersons.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <EKF.hpp>
#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <pcl_ros/impl/transforms.hpp>
#include <sstream>
#include <string>
#include <vector>

struct {
    struct
    {
        float z_threshold = 0.27;  // 平面距离阈值: 点与平面距离小于阈值都视为平面内点
    } filter_ground;
} params;

class agent_tracker {
   private:
    std::map<int, EKF> tracked_agents;

   public:
    struct agent {
        int track_id;
        Eigen::Vector3d p, v;
    };
    agent_tracker() { tracked_agents.clear(); };
    void update_agent_pos(const std::vector<agent>& _agents) {
        std::map<int, EKF> updated_agents;
        for (int i = 0; i < _agents.size(); ++i) {
            const agent& now = _agents[i];
            auto it = tracked_agents.find(now.track_id);
            if (it == tracked_agents.end()) {
                EKF now_ekf;
                now_ekf.reset(now.p);
                updated_agents[now.track_id] = now_ekf;
            } else {
                EKF& now_ekf = it->second;
                now_ekf.update(now.p, 1);
                updated_agents[now.track_id] = now_ekf;
            }
        }
        tracked_agents = updated_agents;
    }
    std::vector<agent> get_agents() {
        std::vector<agent> _agents;
        for (auto it = tracked_agents.begin(); it != tracked_agents.end(); ++it) {
            agent now;
            EKF::Vx& agent_status = it->second.Xe;
            now.track_id = it->first;
            now.p << agent_status[0], agent_status[1], agent_status[2];
            now.v << agent_status[3], agent_status[4], agent_status[5];
            _agents.push_back(now);
        }
        return _agents;
    }
};

tf::TransformListener* tf_listener;
agent_tracker ped_tracker;
tf::StampedTransform base_to_map;
tf::StampedTransform to_baselink;
sensor_msgs::CameraInfo cam_info;
void cam_info_callback(const sensor_msgs::CameraInfoConstPtr& _cam_info_msg) {
    cam_info = *_cam_info_msg;
}
// bbox:[x1 y1 x2 y2] (top left down right)
void extract_obj(const cv::Mat& _depth, const sensor_msgs::CameraInfo& _cam_info, const Eigen::Matrix4f& _transform, const Eigen::Vector4f& _bbox, pcl::PointCloud<pcl::PointXYZ>::Ptr& _points) {
    //相机内参信息
    double fx = _cam_info.K[0];
    double cx = _cam_info.K[2];
    double fy = _cam_info.K[4];
    double cy = _cam_info.K[5];

    _points->clear();
    //获取对应区域点云
    // if(_bbox[0] < 0 || _bbox[2] >= _depth.rows || _bbox[1] < 0 || _bbox[3] >= _depth.cols) return;
    int area_size = (_bbox[3] - _bbox[1]) * (_bbox[2] - _bbox[0]);
    int zero_cnt = 0;
    for (int m = _bbox[1]; m <= _bbox[3]; ++m) {
        for (int n = _bbox[0]; n <= _bbox[2]; ++n) {
            //获得像素点对应深度
            float d = _depth.ptr<uint16_t>(m)[n] * 0.001;
            if (d == 0) {
                ++zero_cnt;
                continue;
            }
            pcl::PointXYZ p;
            //反投影
            p.z = d;
            p.x = (n - cx) * d / fx;
            p.y = (m - cy) * d / fy;
            _points->push_back(p);
        }
    }
    // 60%的区域都没有深度信息，则认为点云无效
    if (zero_cnt > area_size * 0.6) _points->clear();
    //转换坐标系
    pcl::transformPointCloud(*_points, *_points, _transform);
}

void filter_ground(float z_threshold, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filter) {
    cloud_filter->clear();
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    // std::cout << "min_z: " << minPt.z << std::endl;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setNegative(true);
    pass.setFilterLimits(minPt.z, minPt.z + z_threshold);
    pass.filter(*cloud_filter);
}

void filter_shape(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filter) {
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setNegative(false);
    pass.setFilterLimits(minPt.x, minPt.x + 0.5);
    pass.filter(*cloud_filter);
}

void down_sample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filter) {
    //体素滤波
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(0.05f, 0.05f, 0.05f);  //体素网格大小
    vox.filter(*cloud_filter);
}

ros::Publisher detect_obj_pcl_pub;
ros::Publisher bbox_marker_pub;
ros::Publisher ekf_marker_pub;
ros::Publisher agent_pub;
int genp_xs[8] = {0, 3, 3, 0, 0, 3, 3, 0};
int genp_ys[8] = {1, 1, 4, 4, 1, 1, 4, 4};
int genp_zs[8] = {2, 2, 2, 2, 5, 5, 5, 5};
int line_0[12] = {0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3};
int line_1[12] = {1, 2, 3, 0, 5, 6, 7, 4, 4, 5, 6, 7};
void depth_callback(const sensor_msgs::ImageConstPtr& _depth_msg, const mot::MOTConstPtr& _mot_msg) {
    //检查相机参数是否正确
    if (cam_info.K[0] == 0 || cam_info.K[4] == 0) return;

    try {
        //从tf转化成矩阵，获得到车体坐标系的转换关系
        tf_listener->lookupTransform("/base_link", "/fl_cam_color_optical_frame", ros::Time(0), to_baselink);
        //从tf转化成矩阵，获得车体坐标系到map转换关系
        tf_listener->lookupTransform("/map", "/base_link", ros::Time(0), base_to_map);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }
    Eigen::Matrix4f to_baselink_mat, to_map_mat;
    pcl_ros::transformAsMatrix(to_baselink, to_baselink_mat);
    pcl_ros::transformAsMatrix(base_to_map, to_map_mat);

    //获取深度图
    cv_bridge::CvImageConstPtr depth_cvimg = cv_bridge::toCvShare(_depth_msg);
    // cv::imshow("depth", depth_cvimg->image);
    // cv::waitKey(1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr debug_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // 3dbox框
    visualization_msgs::Marker obj_marker;
    obj_marker.header.frame_id = "base_link";
    obj_marker.header.stamp = _depth_msg->header.stamp;
    obj_marker.id = 0;
    obj_marker.action = visualization_msgs::Marker::ADD;
    obj_marker.type = visualization_msgs::Marker::LINE_LIST;
    obj_marker.lifetime = ros::Duration(0.5);
    obj_marker.color.r = 0;
    obj_marker.color.g = 1;
    obj_marker.color.b = 0;
    obj_marker.color.a = 1;
    obj_marker.scale.x = 0.05;
    obj_marker.points.clear();

    // ekf速度信息
    visualization_msgs::Marker ekf_marker;
    ekf_marker.header.frame_id = "map";
    ekf_marker.header.stamp = _depth_msg->header.stamp;
    ekf_marker.id = 0;
    ekf_marker.action = visualization_msgs::Marker::ADD;
    ekf_marker.type = visualization_msgs::Marker::LINE_LIST;
    ekf_marker.lifetime = ros::Duration(0.5);
    ekf_marker.color.r = 0;
    ekf_marker.color.g = 1;
    ekf_marker.color.b = 0;
    ekf_marker.color.a = 1;
    ekf_marker.scale.x = 0.2;
    ekf_marker.points.clear();

    pedsim_msgs::TrackedPersons tracker_msg;
    tracker_msg.header.frame_id = "map";
    tracker_msg.header.stamp = _depth_msg->header.stamp;
    std::vector<agent_tracker::agent> now_peds;
    now_peds.clear();
    for (int i = 0; i < _mot_msg->objs.size(); ++i) {
        if (_mot_msg->objs[i].class_name != "person") continue;

        //从检测框提取点云
        //从depth以及检测信息生成点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        const auto& mot_bbox = _mot_msg->objs[i].bbox;
        Eigen::Vector4f vec_bbox;
        vec_bbox << mot_bbox[0], mot_bbox[1], mot_bbox[2], mot_bbox[3];
        extract_obj(depth_cvimg->image, cam_info, to_baselink_mat, vec_bbox, obj_cloud);
        if (obj_cloud->empty()) continue;
        // // 降采样
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down_sample(new pcl::PointCloud<pcl::PointXYZ>());
        // down_sample(obj_cloud, cloud_down_sample);

        //滤除地面
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_groud_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        filter_ground(params.filter_ground.z_threshold, obj_cloud, filtered_groud_cloud);
        if (filtered_groud_cloud->empty()) continue;

        //去除离群点

        //过滤点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_noise_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        filter_shape(filtered_groud_cloud, filtered_noise_cloud);
        if (filtered_noise_cloud->empty()) continue;

        *debug_cloud += *filtered_noise_cloud;
        //计算质心
        Eigen::Vector4f obj_center;
        pcl::compute3DCentroid(*filtered_groud_cloud, obj_center);

        //转换到map
        Eigen::Vector4f obj_center_map = to_map_mat * obj_center;

        agent_tracker::agent agent_now;
        agent_now.track_id = _mot_msg->objs[i].track_id;
        agent_now.p[0] = obj_center_map[0];
        agent_now.p[1] = obj_center_map[1];
        agent_now.p[2] = 0;
        now_peds.push_back(agent_now);

        //可视化3dbbox
        Eigen::Vector4f minPt, maxPt;
        Eigen::VectorXd vec_indices(6);
        pcl::getMinMax3D(*filtered_noise_cloud, minPt, maxPt);
        vec_indices << minPt[0], minPt[1], minPt[2], maxPt[0], maxPt[1], maxPt[2];
        geometry_msgs::Point bbox_points[8];
        for (int j = 0; j < 8; ++j) {
            bbox_points[j].x = vec_indices[genp_xs[j]];
            bbox_points[j].y = vec_indices[genp_ys[j]];
            bbox_points[j].z = vec_indices[genp_zs[j]];
        }
        for (int j = 0; j < 12; ++j) {
            obj_marker.points.push_back(bbox_points[line_0[j]]);
            obj_marker.points.push_back(bbox_points[line_1[j]]);
        }
    }
    // EKF获得速度信息
    ped_tracker.update_agent_pos(now_peds);
    now_peds = ped_tracker.get_agents();
    for (int i = 0; i < now_peds.size(); ++i) {
        pedsim_msgs::TrackedPerson agent_now;
        agent_now.track_id = _mot_msg->objs[i].track_id;
        agent_now.age = ros::Duration(0);
        agent_now.pose.pose.position.x = now_peds[i].p[0];
        agent_now.pose.pose.position.y = now_peds[i].p[1];
        agent_now.twist.twist.linear.x = now_peds[i].v[0];
        agent_now.twist.twist.linear.y = now_peds[i].v[1];
        tracker_msg.tracks.push_back(agent_now);

        geometry_msgs::Point sp, ep;
        sp.x = now_peds[i].p[0];
        sp.y = now_peds[i].p[1];
        ep.x = now_peds[i].p[0] + now_peds[i].v[0] * 5;
        ep.y = now_peds[i].p[1] + now_peds[i].v[1] * 5;
        ekf_marker.points.push_back(sp);
        ekf_marker.points.push_back(ep);
    }

    //发布调试点云（所有目标的初始点云）
    debug_cloud->height = 1;
    debug_cloud->width = debug_cloud->points.size();
    debug_cloud->is_dense = false;
    sensor_msgs::PointCloud2 debug_global_cloud;
    pcl::toROSMsg(*debug_cloud, debug_global_cloud);
    debug_global_cloud.header.stamp = _depth_msg->header.stamp;
    debug_global_cloud.header.frame_id = "base_link";
    detect_obj_pcl_pub.publish(debug_global_cloud);
    bbox_marker_pub.publish(obj_marker);
    ekf_marker_pub.publish(ekf_marker);
    agent_pub.publish(tracker_msg);
    //
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "estimate_simple");
    ros::NodeHandle nh;
    tf_listener = new tf::TransformListener();
    detect_obj_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/detect_pcl_raw", 10);
    bbox_marker_pub = nh.advertise<visualization_msgs::Marker>("/3dbbox", 10);
    ekf_marker_pub = nh.advertise<visualization_msgs::Marker>("/ekf", 10);
    agent_pub = nh.advertise<pedsim_msgs::TrackedPersons>("/tracked_persons", 10);
    ros::Subscriber cam_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("/fl_cam/aligned_depth_to_color/camera_info", 1, cam_info_callback);

    message_filters::Subscriber<sensor_msgs::Image> cam_depth_sub(
        nh, "/fl_cam/aligned_depth_to_color/image_raw", 1);
    message_filters::Subscriber<mot::MOT> mot_sub(nh, "/objs_detect/mot", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, mot::MOT> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), cam_depth_sub, mot_sub);

    EKF::const_P << 1.0, 1.0, 1.0, 0.6, 0.6, 0.6;
    EKF::const_Q << 1, 1, 1, 10.0, 10.0, 10.0;
    EKF::const_R << 40.0, 40.0, 40.0;

    sync.registerCallback(boost::bind(&depth_callback, _1, _2));
    while (nh.ok()) {
        try {
            tf_listener->waitForTransform("/map", "/fl_cam_color_optical_frame",
                                          ros::Time(0), ros::Duration(3.0));
            tf_listener->lookupTransform("/map", "/fl_cam_color_optical_frame",
                                         ros::Time(0), base_to_map);
            break;
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }
    ros::spin();
}