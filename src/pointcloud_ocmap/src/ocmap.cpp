#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <mot/MOT.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pedsim_msgs/TrackedPersons.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sidewalk_msgs/Trajectories.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_listener.h>

#include <Eigen/Eigen>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl_ros/impl/transforms.hpp>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

struct _param_struct {
    int locfree = 10;
    int lococc = 10;
    int age_thresh = 20;
    int sidewalk_safety = 4;
    int map_width = 20;
    int grid_num_edge = 400;
    int valid_sidewalk_min_point = 50;
    float fill_square = 0.5;
    float blind_area = 1.8;
    std::string tp_pub_goal = "/move_base_simple/goal";
    std::string tp_pub_map = "/occmap";
    std::string tp_pub_map_img = "/occshow";
    std::string tp_pub_odom = "/robot_position";
    std::string tp_sub_mot = "/objs_detect/mot";
    std::string tp_sub_depth = "/fl_cam/aligned_depth_to_color/image_raw";
    std::string tp_sub_depth_cam_info = "/fl_cam/aligned_depth_to_color/camera_info";
    std::string tp_sub_semantic = "/segmentation";
    std::string tp_sub_ped = "/tracked_persons";
    std::string tp_sub_traj = "/prediction_trajectory";
} params;

//在map坐标系下构建，中心点默认（0，0）
class local_occupancy_map {
   public:
    struct ped_loc {
        Eigen::Vector2f loc;
        int track_id;
        int age;
    };

   private:
    struct eigen_hash {
        size_t operator()(const Eigen::Vector2i& p) const {
            return std::hash<int>()(p[0]) ^ std::hash<int>()(p[1]);
        }
    };
    struct eigen_cmp {
        bool operator()(const Eigen::Vector2i& a, const Eigen::Vector2i& b) {
            return a[0] == b[0] ? a[1] < b[1] : a[0] < b[0];
        }
    };
    std::map<Eigen::Vector2i, int, eigen_cmp> grid_points;  //<点，占用概率>
    // std::vector<Eigen::Vector2f> ped_locs;
    std::map<int, ped_loc> ped_locs;
    std::vector<Eigen::Vector3f> traj_locs;
    Eigen::Vector2i robot_pos_grid;
    float map_width;        //正方形局部地图边长（m）
    int grid_num_per_edge;  //一条边上有几个点
    float res;              //地图分辨率
    // bresenham算法获得格点
    std::vector<Eigen::Vector2i> TraceLine(const Eigen::Vector2i& target) {
        Eigen::Vector2i tmpIndex;
        std::vector<Eigen::Vector2i> gridIndexVector;
        int x0 = robot_pos_grid[0];
        int y0 = robot_pos_grid[1];
        int x1 = target[0];
        int y1 = target[1];
        bool steep = abs(y1 - y0) > abs(x1 - x0);
        if (steep) {
            std::swap(x0, y0);
            std::swap(x1, y1);
        }
        if (x0 > x1) {
            std::swap(x0, x1);
            std::swap(y0, y1);
        }

        int deltaX = x1 - x0;
        int deltaY = abs(y1 - y0);
        int error = 0;
        int ystep;
        int y = y0;

        if (y0 < y1) {
            ystep = 1;
        } else {
            ystep = -1;
        }

        int pointX;
        int pointY;
        for (int x = x0; x <= x1; x++) {
            if (steep) {
                pointX = y;
                pointY = x;
            } else {
                pointX = x;
                pointY = y;
            }

            error += deltaY;

            if (2 * error >= deltaX) {
                y += ystep;
                error -= deltaX;
            }

            //不包含最后一个点．
            if (pointX == x1 && pointY == y1) continue;

            //保存所有的点
            tmpIndex[0] = pointX;
            tmpIndex[1] = pointY;

            gridIndexVector.push_back(tmpIndex);
        }
        return gridIndexVector;
    }

   public:
    nav_msgs::OccupancyGrid map_msg;
    cv::Mat map_show;
    local_occupancy_map(int _map_width, int _grid_num_edge) {
        map_width = _map_width;
        grid_num_per_edge = _grid_num_edge;
        res = map_width * 1.0 / grid_num_per_edge;
        map_msg.info.height = _grid_num_edge;
        map_msg.info.width = _grid_num_edge;
        map_msg.info.resolution = res;
        map_msg.info.origin.position.x = 0;
        map_msg.info.origin.position.y = 0;
    }
    Eigen::Vector2i to_grid_index(const Eigen::Vector2f& world_pos) {
        int x_grid = (int)std::round(world_pos[0] / res);
        int y_grid = (int)std::round(world_pos[1] / res);
        return Eigen::Vector2i(x_grid, y_grid);
    }
    Eigen::Vector2f to_world_pos(const Eigen::Vector2i& grid_index) {
        float x_pos = grid_index[0] * res;
        float y_pos = grid_index[1] * res;
        return Eigen::Vector2f(x_pos, y_pos);
    }

    bool check_in_map(const Eigen::Vector2i& grid_index, const Eigen::Vector2i& center) {
        return abs(grid_index[0] - center[0]) < grid_num_per_edge / 2 &&
               abs(grid_index[1] - center[1]) < grid_num_per_edge / 2;
    }
    bool check_in_mat(int img_x, int img_y) {
        return img_x >= 0 && img_x < grid_num_per_edge && img_y >= 0 && img_y < grid_num_per_edge;
    }
    void update_robot_pos(Eigen::Vector2f world_pos) {
        robot_pos_grid = to_grid_index(world_pos);
        //因为是构建局部地图，删除感兴趣区域外的点信息
        for (auto it = grid_points.begin(); it != grid_points.end();) {
            Eigen::Vector2i tmp = it->first;
            if (!check_in_map(tmp, robot_pos_grid)) {
                grid_points.erase(it++);
            } else {
                ++it;
            }
        }
    }
    void update_points(const std::vector<Eigen::Vector2f>& scan_points) {
        std::vector<Eigen::Vector2i> end_points;
        for (int i = 0; i < scan_points.size(); ++i) {
            end_points.push_back(to_grid_index(scan_points[i]));
        }
        std::sort(end_points.begin(), end_points.end(), eigen_cmp());
        end_points.erase(std::unique(end_points.begin(), end_points.end()), end_points.end());
        for (int i = 0; i < end_points.size(); ++i) {
            Eigen::Vector2i& tmp = end_points[i];
            std::vector<Eigen::Vector2i> free_grids = TraceLine(tmp);
            for (int j = 0; j < free_grids.size(); ++j) {
                Eigen::Vector2i free_tmp = free_grids[j];
                if (!check_in_map(free_tmp, robot_pos_grid)) continue;
                auto it = grid_points.find(free_tmp);
                if (it == grid_points.end()) {
                    grid_points[free_tmp] = 50 - params.locfree;
                } else {
                    it->second = std::max(0, it->second - params.locfree);
                }
            }
            if (check_in_map(tmp, robot_pos_grid)) {
                auto it = grid_points.find(tmp);
                if (it == grid_points.end()) {
                    grid_points[tmp] = 50 + params.lococc;
                } else {
                    it->second = std::min(100, it->second + params.lococc);
                }
            }
        }
        //清除age过大的行人
        for (auto it = ped_locs.begin(); it != ped_locs.end();) {
            if (it->second.age > params.age_thresh) {
                ped_locs.erase(it++);
            } else {
                it->second.age++;
                ++it;
            }
        }
    }
    void show_map(const Eigen::Matrix4f& base_to_map) {
        cv::Mat occupancy_local(grid_num_per_edge, grid_num_per_edge, CV_8U, cv::Scalar(50));
        Eigen::Matrix4f map_to_base = base_to_map.inverse();
        int center_index = grid_num_per_edge / 2;
        for (auto it = grid_points.begin(); it != grid_points.end(); ++it) {
            Eigen::Vector2f w_pos = to_world_pos(it->first);
            Eigen::Vector4f tmp_w_pos;
            tmp_w_pos << w_pos[0], w_pos[1], 0, 1;
            Eigen::Vector4f tmp_l_pos = map_to_base * tmp_w_pos;
            Eigen::Vector2i local_grid_index = to_grid_index(Eigen::Vector2f(tmp_l_pos[0], tmp_l_pos[1]));

            int val = it->second;
            if (val < 25) val = 0;
            int img_y = center_index + local_grid_index[1];
            int img_x = center_index + local_grid_index[0];
            if (!check_in_mat(img_x, img_y)) continue;
            occupancy_local.at<uint8_t>(cv::Point2i(img_x, img_y)) = val;
        }
        cv::Mat occupancy_local_blur;
        // cv::medianBlur(occupancy_local,occupancy_local_blur,5);
        cv::bilateralFilter(occupancy_local, occupancy_local_blur, 5, 75, 75);
        for (int i = 0; i < occupancy_local_blur.rows; ++i) {
            uint8_t* ptr = occupancy_local_blur.ptr<uint8_t>(i);
            for (int j = 0; j < occupancy_local_blur.cols; ++j) {
                if (ptr[j] < 30)
                    ptr[j] = 0;
                else
                    ptr[j] = 50;
            }
        }
        //腐蚀边界/安全距离
        if (params.sidewalk_safety > 0) {
            static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(params.sidewalk_safety, params.sidewalk_safety));
            // cv::morphologyEx(sidewalk_path, sidewalk_path, CV_MOP_CLOSE, kernel);
            cv::dilate(occupancy_local_blur, occupancy_local_blur, kernel);
        }

        cv::Mat occ_tmp;
        occ_tmp = (100 - occupancy_local_blur) * 2.5;
        cv::cvtColor(occ_tmp, map_show, CV_GRAY2BGR);

        //显示行人位置
        for (auto it = ped_locs.begin(); it != ped_locs.end(); ++it) {
            Eigen::Vector2f ped_loc = it->second.loc;
            Eigen::Vector4f tmp_ped_loc;
            tmp_ped_loc << ped_loc[0], ped_loc[1], 0, 1;
            Eigen::Vector4f tmp_l_pos = map_to_base * tmp_ped_loc;
            Eigen::Vector2i local_grid_index = to_grid_index(Eigen::Vector2f(tmp_l_pos[0], tmp_l_pos[1]));
            // Eigen::Vector2i tmp = grid_ped_loc - robot_pos_grid;
            int img_y = center_index + local_grid_index[1];
            int img_x = center_index + local_grid_index[0];
            if (!check_in_mat(img_x, img_y)) continue;
            cv::circle(map_show, cv::Point(img_x, img_y), 5, cv::Scalar(0, 0, 255), -1);
            cv::circle(occupancy_local_blur, cv::Point(img_x, img_y), 6, cv::Scalar(50), -1);
            cv::circle(occupancy_local_blur, cv::Point(img_x, img_y), 3, cv::Scalar(100), -1);
        }

        //显示traj
        for (int i = 0; i < traj_locs.size(); ++i) {
            Eigen::Vector2f tmp_loc(traj_locs[i][0], traj_locs[i][1]);
            Eigen::Vector4f tmp_ped_loc;
            tmp_ped_loc << tmp_loc[0], tmp_loc[1], 0, 1;
            Eigen::Vector4f tmp_l_pos = map_to_base * tmp_ped_loc;
            Eigen::Vector2i local_grid_index = to_grid_index(Eigen::Vector2f(tmp_l_pos[0], tmp_l_pos[1]));

            int img_y = center_index + local_grid_index[1];
            int img_x = center_index + local_grid_index[0];
            if (!check_in_mat(img_x, img_y)) continue;
            cv::circle(occupancy_local_blur, cv::Point(img_x, img_y), 2, cv::Scalar(50), -1);
            cv::circle(map_show, cv::Point(img_x, img_y), 3, cv::Scalar(0, 255, 0), -1);
        }

        // cv::Mat occupancy_msg;
        cv::rotate(map_show, map_show, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::flip(map_show, map_show, 1);
        int fill_len = params.fill_square / res / 2;
        int cx = grid_num_per_edge / 2;
        // int cx = 320;
        // int fill_len = 50;
        int cy = cx;
        std::vector<cv::Point> fill_area;
        std::vector<std::vector<cv::Point>> fill_areas;
        fill_area.push_back(cv::Point(cx - fill_len, cy - fill_len));
        fill_area.push_back(cv::Point(cx + fill_len, cy - fill_len));
        fill_area.push_back(cv::Point(cx + fill_len, cy + fill_len));
        fill_area.push_back(cv::Point(cx - fill_len, cy + fill_len));
        fill_areas.push_back(fill_area);
        cv::fillPoly(occupancy_local_blur, fill_areas, cv::Scalar(0));
        map_msg.data = std::vector<int8_t>(occupancy_local_blur.reshape(1, 1));
        // //发布地图
        // cv::resize(show_occ, show_occ, cv::Size(400, 400));
        // cv::imshow("occ", show_occ);
        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", show_occ).toImageMsg();
    }
    void update_ped_loc(const ped_loc& _ped_loc) {
        auto it = ped_locs.find(_ped_loc.track_id);
        if (it == ped_locs.end()) {
            ped_locs[_ped_loc.track_id] = _ped_loc;
            ped_locs[_ped_loc.track_id].age = 0;
        } else {
            it->second.loc = _ped_loc.loc;
            it->second.age = 0;
        }
    }
    void set_traj_locations(const std::vector<Eigen::Vector3f>& _traj_locs) {
        traj_locs = _traj_locs;
    }
};

ros::Publisher occmap_pub;
ros::Publisher occshow_pub;
ros::Publisher robotpos_pub;
ros::Publisher localgoal_pub;
nav_msgs::Odometry robotpos_msg;
geometry_msgs::PoseStamped localgoal_msg;
sensor_msgs::ImagePtr occshow_msg;
tf::TransformListener* tf_listener;
tf::StampedTransform cam_to_base;
tf::StampedTransform base_to_map;
local_occupancy_map* local_map;
void cam_callback(const sensor_msgs::ImageConstPtr& _depth_img,
                  const sensor_msgs::CameraInfoConstPtr& _cam_info,
                  const sensor_msgs::ImageConstPtr& _semantic_img,
                  const mot::MOTConstPtr& _mot_info) {
    sensor_msgs::Image img_cp = *_semantic_img;
    // std::cout << img_cp.data.size() << std::endl;
    // std::cout << img_cp.encoding << std::endl;
    img_cp.encoding = "mono8";
    // img_cp.step
    //相机内参获取
    double fx = _cam_info->K[0];
    double cx = _cam_info->K[2];
    double fy = _cam_info->K[4];
    double cy = _cam_info->K[5];
    // ROS_WARN("%s", "map enter!");
    try {
        //到base_link的转换
        tf_listener->lookupTransform("/base_link", "/fl_cam_color_optical_frame",
                                     ros::Time(0), cam_to_base);
        tf_listener->lookupTransform("/map", "/base_link", ros::Time(0), base_to_map);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }

    Eigen::Matrix4f mat_cam_to_base, mat_base_to_map;
    //从tf转化成矩阵
    pcl_ros::transformAsMatrix(cam_to_base, mat_cam_to_base);
    pcl_ros::transformAsMatrix(base_to_map, mat_base_to_map);
    //当前机器人世界坐标
    Eigen::Vector2f now_robot_pos;
    now_robot_pos[0] = mat_base_to_map(0, 3);
    now_robot_pos[1] = mat_base_to_map(1, 3);
    robotpos_msg.header.frame_id = "map";
    robotpos_msg.header.stamp = _depth_img->header.stamp;
    robotpos_msg.pose.pose.position.x = now_robot_pos[0];
    robotpos_msg.pose.pose.position.y = now_robot_pos[1];
    robotpos_pub.publish(robotpos_msg);
    // std::cout << "Robot_Pos: " << now_robot_pos << std::endl;
    // 获得语义分割以及深度图
    // int L = 640;  // 10m对应L
    // cv::Mat occupancy(L, L, CV_8U, cv::Scalar(0));
    cv::Mat sidewalk_path, depth_zero;
    cv_bridge::CvImageConstPtr cvimg = cv_bridge::toCvShare(_depth_img);
    cv_bridge::CvImagePtr sematic_img = cv_bridge::toCvCopy(img_cp);
    cv::inRange(sematic_img->image, 1, 1, sidewalk_path);
    cv::Mat contour_img(sidewalk_path.rows, sidewalk_path.cols, CV_8U, cv::Scalar(0));
    cv::Mat contour_filter_img(sidewalk_path.rows, sidewalk_path.cols, CV_8U, cv::Scalar(0));
    //获取深度图像为0的区域
    cv::inRange(cvimg->image, 0, 0, depth_zero);
    // cv::imshow("d0",depth_zero);
    sidewalk_path -= depth_zero;

    //对语义分割进行闭+开操作
    static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
    cv::morphologyEx(sidewalk_path, sidewalk_path, CV_MOP_CLOSE, kernel);
    cv::morphologyEx(sidewalk_path, sidewalk_path, CV_MOP_OPEN, kernel);
    // std::cout << sematic_img->image.type() << std::endl;
    // cv::imshow("depth", cvimg->image);
    // cv::imshow("sidewalk", sidewalk_path);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(sidewalk_path, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    //找最大轮廓
    int contours_size_max = 0;
    int contour_max_index = 0;
    for (int i = 0; i < contours.size(); ++i) {
        if (contours[i].size() > contours_size_max) {
            contour_max_index = i;
            contours_size_max = contours[i].size();
        }
    }
    if (contours_size_max <= params.valid_sidewalk_min_point) {
        ROS_ERROR("no valid sidewalk area!");
    } else {
        cv::drawContours(contour_img, contours, contour_max_index, cv::Scalar(255), 2);
        for (int i = 0; i < _mot_info->objs.size(); ++i) {
            const mot::MOT_OBJ& tmp_obj = _mot_info->objs[i];
            if (tmp_obj.class_name != "person") continue;
            cv::rectangle(contour_img,
                          cv::Rect(cv::Point(tmp_obj.bbox[0], tmp_obj.bbox[1]),
                                   cv::Point(tmp_obj.bbox[2], tmp_obj.bbox[3])),
                          cv::Scalar(255));
        }
        std::vector<cv::Point>& max_contour = contours[contour_max_index];
        std::vector<Eigen::Vector4f> contour_3d;
        std::vector<Eigen::Vector2f> contour_2d;
        for (int i = 0; i < max_contour.size(); ++i) {
            cv::Point now = max_contour[i];
            //处理行人信息，修正sidewalk信息
            bool point_in_obj = false;
            for (int i = 0; i < _mot_info->objs.size(); ++i) {
                const mot::MOT_OBJ& tmp_obj = _mot_info->objs[i];
                if (tmp_obj.class_name != "person") continue;
                //判断点在被影响的区域内
                if (now.x > tmp_obj.bbox[0] - 5 && now.x < tmp_obj.bbox[2] + 5 && now.y > tmp_obj.bbox[1] - 12 && now.y < tmp_obj.bbox[3] + 12) {
                    point_in_obj = true;
                    break;
                }
            }
            if (point_in_obj) continue;
            cv::circle(contour_filter_img, now, 1, cv::Scalar(255));
            float d = cvimg->image.at<uint16_t>(now) * 0.001;
            if (d == 0) continue;
            Eigen::Vector4f p;
            p[0] = (now.x - cx) * d / fx;
            p[1] = (now.y - cy) * d / fy;
            p[2] = d;
            p[3] = 1;
            //转换到base_link
            Eigen::Vector4f p_base = mat_cam_to_base * p;
            contour_3d.push_back(p_base);
        }

        for (int i = 0; i < contour_3d.size(); ++i) {
            // base_link x前 y左
            // opencv x右 y下
            // 去除盲区
            if (contour_3d[i][0] >= params.blind_area) {
                //转到map坐标系
                Eigen::Vector4f p_map = mat_base_to_map * contour_3d[i];
                contour_2d.push_back(Eigen::Vector2f(p_map[0], p_map[1]));
            }
        }
        // std::cout << "x_min: " << x_min << std::endl;
        std::chrono::system_clock::time_point bg = std::chrono::system_clock::now();
        local_map->update_robot_pos(now_robot_pos);
        local_map->update_points(contour_2d);
        local_map->show_map(mat_base_to_map);
        std::chrono::system_clock::time_point ed = std::chrono::system_clock::now();
        std::cout << "occmap building:" << std::chrono::duration_cast<std::chrono::milliseconds>(ed - bg).count() << " milliseconds." << std::endl;
        nav_msgs::OccupancyGrid& map_msg = local_map->map_msg;
        map_msg.header.frame_id = "base_link";
        map_msg.header.stamp = _depth_img->header.stamp;
        map_msg.info.origin.position.x = -10;
        map_msg.info.origin.position.y = -10;
        occmap_pub.publish(map_msg);

        occshow_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", local_map->map_show).toImageMsg();
        occshow_pub.publish(occshow_msg);

        localgoal_msg.pose.position.x = 6;
        localgoal_msg.pose.position.y = 0;
        localgoal_msg.pose.position.z = 0;
        localgoal_msg.pose.orientation.x = 0;
        localgoal_msg.pose.orientation.y = 0;
        localgoal_msg.pose.orientation.z = 0;
        localgoal_msg.pose.orientation.w = 1;
        localgoal_msg.header.frame_id = "base_link";
        localgoal_msg.header.stamp = _depth_img->header.stamp;
        // localgoal_pub.publish(localgoal_msg);
    }
    // cv::imshow("con", contour_img);
    // cv::imshow("filter",contour_filter_img);
    // cv::imshow("map", occupancy);
    // cv::waitKey(1);
}

void ped_update_callback(const pedsim_msgs::TrackedPersonsConstPtr& _ped_info) {
    if (local_map == NULL) return;
    for (int i = 0; i < _ped_info->tracks.size(); ++i) {
        const pedsim_msgs::TrackedPerson& tmp_obj = _ped_info->tracks[i];
        float loc_x = tmp_obj.pose.pose.position.x;
        float loc_y = tmp_obj.pose.pose.position.y;
        std::cout << "ped: x: " << loc_x << " y:" << loc_y;
        if (loc_x == 0 && loc_y == 0) continue;
        local_occupancy_map::ped_loc ped_now;
        ped_now.loc = Eigen::Vector2f(loc_x, loc_y);
        ped_now.age = 0;
        ped_now.track_id = tmp_obj.track_id;
        local_map->update_ped_loc(ped_now);
    }
}

void traj_update_callback(const sidewalk_msgs::TrajectoriesConstPtr& traj_info) {
    std::vector<Eigen::Vector3f> now_traj;
    for (int i = 0; i < traj_info->trajectories.size(); ++i) {
        const auto& tmp_traj = traj_info->trajectories[i];
        for (int j = 0; j < tmp_traj.position.size(); ++j) {
            const auto& tmp_pos = tmp_traj.position[j];
            now_traj.push_back(Eigen::Vector3f(tmp_pos.x, tmp_pos.y, 1));
        }
        std::cout << "traj size:" << tmp_traj.position.size() << std::endl;
    }
    if (local_map != NULL) {
        local_map->set_traj_locations(now_traj);
    }
}

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, mot::MOT>
    SyncPolicy;

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_occupancy_map");
    ros::NodeHandle nh("~");

    nh.param("locfree",params.locfree,{10});
    nh.param("lococc",params.lococc,{10});
    nh.param("age_thresh",params.age_thresh,{20});
    nh.param("sidewalk_safety",params.sidewalk_safety,{4});
    nh.param("map_width",params.map_width,{20});
    nh.param("grid_num_edge",params.grid_num_edge,{400});
    nh.param("valid_sidewalk_min_point",params.valid_sidewalk_min_point,{50});
    nh.param("fill_square",params.fill_square,{0.5});
    nh.param("blind_area",params.blind_area,{1.8});
    nh.param("tp_pub_goal",params.tp_pub_goal,{"pub_goal"});
    nh.param("tp_pub_map",params.tp_pub_map,{"pub_map"});
    nh.param("tp_pub_map_img",params.tp_pub_map_img,{"pub_map_img"});
    nh.param("tp_pub_odom",params.tp_pub_odom,{"pub_odom"});
    nh.param("tp_sub_mot",params.tp_sub_mot,{"sub_mot"});
    nh.param("tp_sub_depth",params.tp_sub_depth,{"sub_depth"});
    nh.param("tp_sub_depth_cam_info",params.tp_sub_depth_cam_info,{"sub_depth_cam_info"});
    nh.param("tp_sub_semantic",params.tp_sub_semantic,{"sub_semantic"});
    nh.param("tp_sub_ped",params.tp_sub_ped,{"sub_ped"});
    nh.param("tp_sub_traj",params.tp_sub_traj,{"sub_traj"});

    // pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/fl_cam/depth_pc2", 10);
    // pcl_sidewalk_pub =
    //     nh.advertise<sensor_msgs::PointCloud2>("/sidewalk_pc2", 10);
    occmap_pub = nh.advertise<nav_msgs::OccupancyGrid>(params.tp_pub_map, 10);
    robotpos_pub = nh.advertise<nav_msgs::Odometry>(params.tp_pub_odom, 10);
    localgoal_pub = nh.advertise<geometry_msgs::PoseStamped>(params.tp_pub_goal, 10);
    occshow_pub = nh.advertise<sensor_msgs::Image>(params.tp_pub_map_img, 1);
    ros::Subscriber ped_sub = nh.subscribe<pedsim_msgs::TrackedPersons>(params.tp_sub_ped, 1, ped_update_callback);
    ros::Subscriber traj_sub = nh.subscribe<sidewalk_msgs::Trajectories>(params.tp_sub_traj, 1, traj_update_callback);
    tf_listener = new tf::TransformListener();
    message_filters::Subscriber<sensor_msgs::Image> cam_depth_sub(nh, params.tp_sub_depth, 1);
    message_filters::Subscriber<sensor_msgs::Image> cam_sematic(nh, params.tp_sub_semantic, 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub(nh, params.tp_sub_depth_cam_info, 1);
    message_filters::Subscriber<mot::MOT> mot_sub(nh, params.tp_sub_mot, 1);
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), cam_depth_sub, cam_info_sub, cam_sematic, mot_sub);
    sync.registerCallback(boost::bind(&cam_callback, _1, _2, _3, _4));
    local_map = new local_occupancy_map(params.map_width, params.grid_num_edge);
    while (nh.ok()) {
        try {
            tf_listener->waitForTransform("/map", "/base_link",
                                          ros::Time(0), ros::Duration(3.0));
            tf_listener->lookupTransform("/map", "/base_link",
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