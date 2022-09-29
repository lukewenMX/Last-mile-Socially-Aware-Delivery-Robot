#ifndef COMMON_VIEW_H
#define COMMON_VIEW_H

#include "common.h"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <Eigen/Eigen>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>  //  //PCL可视化的头文件

class CommonView
{
public:
  /** @brief 绘制3DBBox
   *  @param viewer in PCL绘制容器
   *  @param coeff in 3dbox系数(自定义): [x_min, y_min, z_min, x_max, y_max, z_max]
   *  @param color int 3dbox线条颜色
   *  @param id_prefix in 图元ID前缀
   *  @param viewport in 绘制窗口ID
   *  @return bool
   */
  static bool draw_3dbbox(pcl::visualization::PCLVisualizer::Ptr viewer, const Eigen::Matrix<float, 1, 9> &bbox3d, int color,
                   const std::string &id_prefix, int viewport)
  {
    // 绘制3DBox
    char id[64] = {0};
    float x = bbox3d[0];
    float y = bbox3d[1];
    float z = bbox3d[2];
    //float R = bbox3d[3];
    //float P = bbox3d[4];
    //float Y = bbox3d[5];
    float L = bbox3d[6];
    float W = bbox3d[7];
    float H = bbox3d[8];
    float x_min = x - L/2;
    float y_min = y - W/2;
    float z_min = z - H/2;
    float x_max = x + L/2;
    float y_max = y + W/2;
    float z_max = z + H/2;
    float r = (float)((color&0x00FF0000)>>16)/255;
    float g = (float)((color&0x0000FF00)>>8)/255;
    float b = (float)((color&0x000000FF)>>0)/255;
    // 坐标系: 世界坐标系
    // 点排序: 先底面后顶面，逆时针
    pcl::PointXYZ p1(x_min, y_min, z_min);
    pcl::PointXYZ p2(x_max, y_min, z_min);
    pcl::PointXYZ p3(x_max, y_max, z_min);
    pcl::PointXYZ p4(x_min, y_max, z_min);
    pcl::PointXYZ p5(x_min, y_min, z_max);
    pcl::PointXYZ p6(x_max, y_min, z_max);
    pcl::PointXYZ p7(x_max, y_max, z_max);
    pcl::PointXYZ p8(x_min, y_max, z_max);

    sprintf(id, "%s_%d_1", id_prefix.c_str(), color);
    viewer->addLine(p1, p2, r, g, b, id, viewport);
    sprintf(id, "%s_%d_2", id_prefix.c_str(), color);
    viewer->addLine(p2, p3, r, g, b, id, viewport);
    sprintf(id, "%s_%d_3", id_prefix.c_str(), color);
    viewer->addLine(p3, p4, r, g, b, id, viewport);
    sprintf(id, "%s_%d_4", id_prefix.c_str(), color);
    viewer->addLine(p4, p1, r, g, b, id, viewport);
    sprintf(id, "%s_%d_5", id_prefix.c_str(), color);
    viewer->addLine(p5, p6, r, g, b, id, viewport);
    sprintf(id, "%s_%d_6", id_prefix.c_str(), color);
    viewer->addLine(p6, p7, r, g, b, id, viewport);
    sprintf(id, "%s_%d_7", id_prefix.c_str(), color);
    viewer->addLine(p7, p8, r, g, b, id, viewport);
    sprintf(id, "%s_%d_8", id_prefix.c_str(), color);
    viewer->addLine(p8, p5, r, g, b, id, viewport);
    sprintf(id, "%s_%d_9", id_prefix.c_str(), color);
    viewer->addLine(p1, p5, r, g, b, id, viewport);
    sprintf(id, "%s_%d_10", id_prefix.c_str(), color);
    viewer->addLine(p2, p6, r, g, b, id, viewport);
    sprintf(id, "%s_%d_11", id_prefix.c_str(), color);
    viewer->addLine(p3, p7, r, g, b, id, viewport);
    sprintf(id, "%s_%d_12", id_prefix.c_str(), color);
    viewer->addLine(p4, p8, r, g, b, id, viewport);

    sprintf(id, "%s_%d_c", id_prefix.c_str(), color);
    //float center_x = (x_min+x_max)/2;
    //float center_y = (y_min+y_max)/2;
    //float center_z = (z_min+z_max)/2;
    //float size_x = x_max - x_min;
    //float size_y = y_max - y_min;
    //float size_z = z_max - z_min;

    //viewer->addText3D("c", PointT(center_x, center_y, center_z), 0.1, b, g, r, id, viewport);
    viewer->addCube((x_min+x_max)/2-0.025, (x_min+x_max)/2+0.025,
                    (y_min+y_max)/2-0.025, (y_min+y_max)/2+0.025,
                    (z_min+z_max)/2-0.025, (z_min+z_max)/2+0.025,
                    r, g, b, id, viewport);

    //printf("[draw_3dbbox] color=%d, bbox(%f,%f,%f,%f,%f,%f), center(%f, %f, %f), size(%f, %f, %f)\n", color,
    //       x_min, y_min, z_min, x_max, y_max, z_max, center_x, center_y, center_z, size_x, size_y, size_z);
    //viewer.addCube(x_min, x_max, y_min, y_max, z_min, z_max, r, g, b, id, viewport); // 不透明
    return true;
  }


  static bool draw_3dbbox2(pcl::visualization::PCLVisualizer::Ptr viewer, const Eigen::Matrix<float, 1, 9> &bbox3d, int color,
                   const std::string &id_prefix, int viewport)
  {
    // 中心在原点
    // =========
    float x = 0;
    float y = 0;
    float z = 0;
    float L = bbox3d[6];
    float W = bbox3d[7];
    float H = bbox3d[8];
    float x_min = x - L/2;
    float y_min = y - W/2;
    float z_min = z - H/2;
    float x_max = x + L/2;
    float y_max = y + W/2;
    float z_max = z + H/2;
    Eigen::Vector3f o_p1(x_min, y_min, z_min);
    Eigen::Vector3f o_p2(x_max, y_min, z_min);
    Eigen::Vector3f o_p3(x_max, y_max, z_min);
    Eigen::Vector3f o_p4(x_min, y_max, z_min);
    Eigen::Vector3f o_p5(x_min, y_min, z_max);
    Eigen::Vector3f o_p6(x_max, y_min, z_max);
    Eigen::Vector3f o_p7(x_max, y_max, z_max);
    Eigen::Vector3f o_p8(x_min, y_max, z_max);

    // 变换矩阵
    // =========
    Eigen::Matrix<float, 1, 6> pose;
    pose << bbox3d[0], bbox3d[1], bbox3d[2], bbox3d[3], bbox3d[4], bbox3d[5];
    Eigen::Matrix4f T = CommonFuns::pose2T(pose);
    Eigen::Affine3f affine;
    affine.matrix()= T;

    // 坐标变换
    // =========
    Eigen::Vector3f a_p1;
    Eigen::Vector3f a_p2;
    Eigen::Vector3f a_p3;
    Eigen::Vector3f a_p4;
    Eigen::Vector3f a_p5;
    Eigen::Vector3f a_p6;
    Eigen::Vector3f a_p7;
    Eigen::Vector3f a_p8;
    pcl::transformPoint(o_p1, a_p1, affine);
    pcl::transformPoint(o_p2, a_p2, affine);
    pcl::transformPoint(o_p3, a_p3, affine);
    pcl::transformPoint(o_p4, a_p4, affine);
    pcl::transformPoint(o_p5, a_p5, affine);
    pcl::transformPoint(o_p6, a_p6, affine);
    pcl::transformPoint(o_p7, a_p7, affine);
    pcl::transformPoint(o_p8, a_p8, affine);

    pcl::PointXYZ p1(a_p1[0], a_p1[1], a_p1[2]);
    pcl::PointXYZ p2(a_p2[0], a_p2[1], a_p2[2]);
    pcl::PointXYZ p3(a_p3[0], a_p3[1], a_p3[2]);
    pcl::PointXYZ p4(a_p4[0], a_p4[1], a_p4[2]);
    pcl::PointXYZ p5(a_p5[0], a_p5[1], a_p5[2]);
    pcl::PointXYZ p6(a_p6[0], a_p6[1], a_p6[2]);
    pcl::PointXYZ p7(a_p7[0], a_p7[1], a_p7[2]);
    pcl::PointXYZ p8(a_p8[0], a_p8[1], a_p8[2]);

    // 绘制3DBox
    char id[64] = {0};
    float r = (float)((color&0x00FF0000)>>16)/255;
    float g = (float)((color&0x0000FF00)>>8)/255;
    float b = (float)((color&0x000000FF)>>0)/255;

    sprintf(id, "%s_%d_1", id_prefix.c_str(), color);
    viewer->addLine(p1, p2, r, g, b, id, viewport);
    sprintf(id, "%s_%d_2", id_prefix.c_str(), color);
    viewer->addLine(p2, p3, r, g, b, id, viewport);
    sprintf(id, "%s_%d_3", id_prefix.c_str(), color);
    viewer->addLine(p3, p4, r, g, b, id, viewport);
    sprintf(id, "%s_%d_4", id_prefix.c_str(), color);
    viewer->addLine(p4, p1, r, g, b, id, viewport);
    sprintf(id, "%s_%d_5", id_prefix.c_str(), color);
    viewer->addLine(p5, p6, r, g, b, id, viewport);
    sprintf(id, "%s_%d_6", id_prefix.c_str(), color);
    viewer->addLine(p6, p7, r, g, b, id, viewport);
    sprintf(id, "%s_%d_7", id_prefix.c_str(), color);
    viewer->addLine(p7, p8, r, g, b, id, viewport);
    sprintf(id, "%s_%d_8", id_prefix.c_str(), color);
    viewer->addLine(p8, p5, r, g, b, id, viewport);
    sprintf(id, "%s_%d_9", id_prefix.c_str(), color);
    viewer->addLine(p1, p5, r, g, b, id, viewport);
    sprintf(id, "%s_%d_10", id_prefix.c_str(), color);
    viewer->addLine(p2, p6, r, g, b, id, viewport);
    sprintf(id, "%s_%d_11", id_prefix.c_str(), color);
    viewer->addLine(p3, p7, r, g, b, id, viewport);
    sprintf(id, "%s_%d_12", id_prefix.c_str(), color);
    viewer->addLine(p4, p8, r, g, b, id, viewport);

    return true;
  }
};

#endif // COMMON_VIEW_H
