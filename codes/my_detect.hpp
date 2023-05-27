#pragma once
#ifndef __MY_DETECT_HPP__
#define __MY_DETECT_HPP__
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "open3d/Open3D.h"
#include <Eigen/Geometry>
using namespace std;
using namespace open3d;
using PcdPtr = shared_ptr<geometry::PointCloud>;
using OBBX = geometry::OrientedBoundingBox;
using ABBX = geometry::AxisAlignedBoundingBox;
/*———————————————————————————————————————————————————————————————————————————
// 函数申明
———————————————————————————————————————————————————————————————————————————*/
bool is_rotation_matrix(Eigen::Matrix3d R);
double get_angle(Eigen::Vector3d v1, Eigen::Vector3d v2);
Eigen::Vector3d mean_point(vector<Eigen::Vector3d> p);
Eigen::Vector3d get_rpy_from_rotation_matrix(Eigen::Matrix3d& R);
Eigen::Vector3d get_rpy_form_rotation_vector(Eigen::Vector3d rv);
Eigen::Vector3d get_rotation_vector_from_rpy(Eigen::Vector3d rpy);
double point_to_plane_distance(Eigen::Vector3d p, Eigen::Vector4d c);
/*———————————————————————————————————————————————————————————————————————————
// 封装 检测的类
———————————————————————————————————————————————————————————————————————————*/
class MyDetect
{
public:
	MyDetect();
	~MyDetect();
	bool load_pcd_from_file_or_camera(bool is_camera, PcdPtr pcd, string file_name);
	void uniform_down_filter_src(int k=5);
	void split_pcd_by_cluster_dbscan_src(double r = 3, int min_points = 5);
	void get_outer_pcd_by_points_nums(int min_points = 650, int max_points = 1650);
	void get_outer_pcd_by_points_obbx(double min_area=50, double max_area=70);
	void fit_outer_plane();
	void get_inner_pcd_by_expend_out_obbx(double kx=1.0,double ky=1.0,double kz=6.5);
	void get_inner_pcd_by_cluster(double r = 5.0, int min_points = 5);
	void get_inner_direction_by_cluster(double r,double min_points);
	void get_outer_direction_by_iterate_abbx(double len = 15.0);
	void caculate_pose();
	void vis_geometry();
	void clear_all();
	// 添加所有函数
	void pross_pcd(bool is_camera, PcdPtr pcd, string str);
public:
	vector<vector<double>> ur_move_pose; 	// 外部访问 赋值
    Eigen::Matrix4d pub_cam2obj;
private:
	visualization::Visualizer vis; // 含义见命名
	
	vector<vector<size_t> > all_cluster, obj_cluster; //簇的索引
	PcdPtr src_pcd_ptr,
		dst_pcd_ptr,
		uni_pcd_ptr,
		inner_pcd_ptr,
		outer_pcd_ptr;
	OBBX outer_pcd_obbx, inner_pcd_obbx,
		outer_roi_obbx,inner_center_obbx;
	vector<PcdPtr> obj_pcds;
	Eigen::Vector3d
		n0, nx, ny, nz,
		inner_nx, inner_ny,
		inner_cp2, inner_cp7,
		outer_nx, outer_ny,
		inner_cp, outer_cp,
		path_cp; // 路径点
	Eigen::Vector4d outer_plane_coef;
	Eigen::Matrix4d
        base2cam, cam2base,
		cam2obj,  cam2path,
		base2end, end2base,
		end2tool, tool2end,
		ab_base2obj,
		re_base2obj;
};
#endif
