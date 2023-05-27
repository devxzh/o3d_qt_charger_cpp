/*————————————————————————————————————————————————————————
Win11 + VS2019 + Open3D 0.15.2
充电口识别 open3d c++ 版本 
create by dorob@qq.com 2022-05-16

机械臂IP和相机IP 不在同一IP范围内, 
相机 192.168.1.67
UR3 192.168.3.20 
UR5 192.168.56.101
需要修改一个,注意修改后复原
使用USB转网口,且需要手动分配IP
—————————————————————————————————————————————————————————*/
#pragma once
#pragma comment(lib, "libscan_release.lib")

// 注意ur-rtde的头文件要在 scanner_api 前面
// 否则 会因为 winSock.h 重复包含报错
// https://blog.csdn.net/gzlyb/article/details/5870326
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <iostream>
#include <memory>
#include <thread>
#include <ctime>
#include <string>
#include <conio.h> // getch()
#include <vector>
#include <iomanip>  // cout format


#include <open3d/Open3D.h>
#include "scanner_api.h"
#include <Eigen/Geometry>


using namespace std;
using namespace open3d;
using namespace ur_rtde;

time_t now = time(0); 

/*—————————————————————————————————————————————————————————
 global variable
—————————————————————————————————————————————————————————*/
std::vector<DEVICE_INFO> g_Device_list;   //设备列表
char g_Mac_address[256] = { 0 };		  //MAC地址
char g_Server_IP[256] = "192.168.1.67"; //相机IP
char g_Device_ID[256] = "COMD0511001";    //相机ID
char g_Net_mask[256] = "255.255.255.0";   //子网掩码

std::shared_ptr<geometry::PointCloud> g_pcd_ptr(new geometry::PointCloud);

bool g_bConnected = false;		//连接与否
bool g_bReconnect = false;		//重连标志位
bool g_scan_status = true;		//扫描完成状态
Scanner* g_pScanner = NULL;		//相机扫描仪

//auto& vis = visualization::Visualizer::Visualizer(); // 非阻塞窗口

vector<vector<double>> g_pose;
string g_ensure = "";

/*—————————————————————————————————————————————————————————
//获取任意长度的数字输入。滤掉非数字
// string.c_str()获取指针地址，atoi转换为整型
—————————————————————————————————————————————————————————*/
#define DLP_STD_CIN_GET(value)   \
    {                            \
        std::string x;           \
        std::cin >> x;           \
        value = atoi(x.c_str()); \
    }


#define SPACE_KEY (32) //定义空格键
#define ESC_KEY (27)   //定义ESC键


/*—————————————————————————————————————————————————————————
// 点击连接后，改变标志位，输出连接信息
—————————————————————————————————————————————————————————*/
void ConnectedCallfunc(bool connected)
{
	g_bConnected = connected;
	printf_s("ConnectedCallfunc (connected=%d)! \n", connected);
	return;
}

/*—————————————————————————————————————————————————————————
//回调函数, 获取设备信息
—————————————————————————————————————————————————————————*/
int DiscoveryCallfunc(DISCOVERY_TYPE_E type, char* dataAddr, int dataSize)
{
	char* start = NULL; 
	char* end = NULL;
	DEVICE_INFO stDevice;
	memset(&stDevice, 0, sizeof(DEVICE_INFO)); //设置设备信息为0

	if ((start = strstr(dataAddr, "deviceid:\"")) != NULL) //查找第一次出现deviceid的位置
	{
		start += strlen("deviceid:\"");
		end = strstr(start, "\"");

		strncpy_s(stDevice.deviceid, start, end - start); //获取设备ID
	}

	if ((start = strstr(dataAddr, "ipaddr:\"")) != NULL)
	{
		start += strlen("ipaddr:\"");
		end = strstr(start, "\"");

		strncpy_s(stDevice.ip, start, end - start); //获取设备IP
	}

	if ((start = strstr(dataAddr, "hwadd:\"")) != NULL)
	{
		start += strlen("hwadd:\"");
		end = strstr(start, "\"");

		strncpy_s(stDevice.mac, start, end - start); //获取设备MAC
	}

	if ((start = strstr(dataAddr, "status:\"")) != NULL)
	{
		start += strlen("status:\"");
		end = strstr(start, "\"");

		strncpy_s(stDevice.status, start, end - start); //获取设备状态
	}

	if ((start = strstr(dataAddr, "sdkversion:\"")) != NULL)
	{
		start += strlen("sdkversion:\"");
		end = strstr(start, "\"");

		strncpy_s(stDevice.sdkversion, start, end - start); //获取SDK版本
	}

	bool find = false;
	for (int i = 0; i < g_Device_list.size(); i++)
	{
		if (0 == strcmp(stDevice.mac, g_Device_list[i].mac)) // mac地址相同时
		{
			strncpy_s(g_Device_list[i].ip, stDevice.ip, strlen(stDevice.ip) + 1);
			strncpy_s(g_Device_list[i].status, stDevice.status, strlen(stDevice.status) + 1);
			strncpy_s(g_Device_list[i].sdkversion, stDevice.sdkversion, strlen(stDevice.sdkversion) + 1);
			strncpy_s(g_Device_list[i].deviceid, stDevice.deviceid, strlen(stDevice.deviceid) + 1);
			find = true;
			break;
		}
	}

	if (!find) //如果没有重复设备，就添加
	{
		g_Device_list.push_back(stDevice);
	}

	return 0;
}



/*—————————————————————————————————————————————————————————
//获取数据流,返回点云指针,或保存io::write
—————————————————————————————————————————————————————————*/
std::shared_ptr<geometry::PointCloud> get_pointCloud(char* pData, int dataSize)
{
	std::shared_ptr<geometry::PointCloud> pcd_ptr(new geometry::PointCloud);

	float* pPointCloud = (float*)pData;
	int count = dataSize / (3 * sizeof(float));

	for (int i = 0; i < count; i++)
	{
		float x = (float)pPointCloud[i * 3 + 0];
		float y = (float)pPointCloud[i * 3 + 1];
		float z = (float)pPointCloud[i * 3 + 2];
		pcd_ptr->points_.push_back(Eigen::Vector3d(x, y, z));
	}
	return pcd_ptr;
}


/*—————————————————————————————————————————————————————————
// 回调函数,执行图像采集,这里仅获取xyz坐标
—————————————————————————————————————————————————————————*/
int StreamCallfunc(STREAM_TYPE_E type, char* dataAddr, int dataSize)
{
	if (type == STREAM_TYPE_SCAN_ERROR)
	{
		printf_s("[Camera Info:] STREAM_TYPE_SCAN_ERROR \n");
	}
	if (type == STREAM_TYPE_SCAN_OVER)
	{
		g_scan_status = true;
		printf_s("[Camera Info:] STREAM_TYPE_SCAN_OVER \n");
	}
	else if (type == STREAM_TYPE_POINT_CLOUD)//1
	{
		g_pcd_ptr = get_pointCloud(dataAddr, dataSize);
		printf_s("[Camera Info:] STREAM_TYPE_POINT_CLOUD: dataSize=%d \n", dataSize);
	}

	return 0;
}


/*—————————————————————————————————————————————————————————
计算点到平面的距离,点坐标,平面系数
—————————————————————————————————————————————————————————*/
double point_to_plane_distance(Eigen::Vector3d p, Eigen::Vector4d c)
{
	double da = std::abs(c[0] * p[0] + c[1] * p[1] + c[2] * p[2] + c[3]);
	double ds = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] + c[2]);
	return da / ds;
}

/*—————————————————————————————————————————————————————————
计算坐标平均值
—————————————————————————————————————————————————————————*/
Eigen::Vector3d mean_point(vector<Eigen::Vector3d> p)
{
	Eigen::Vector3d p_sum{0,0,0};
	for (int i = 0; i < p.size(); i++)
	{
		p_sum += p[i];
	}
	return p_sum / p.size();
}

/*—————————————————————————————————————————————————————————
计算向量夹角,返回弧度
—————————————————————————————————————————————————————————*/
double get_angle(Eigen::Vector3d v1,Eigen::Vector3d v2)
{
	double cosAlpha = v1.dot(v2) / (v1.norm() * v2.norm());
	double rad = acos(cosAlpha) * 180 / 3.1415926;
	return rad;
}

/*—————————————————————————————————————————————————————————
判断是否为旋转矩阵 R*Rt=I
—————————————————————————————————————————————————————————*/
bool is_rotation_matrix(Eigen::Matrix3d R)
{
	auto I_mat = R * R.transpose();
	auto eye3 = Eigen::MatrixXd::Identity(3, 3);
	double error = (eye3 - I_mat).norm();

	/*cout << I_mat << endl;
	cout << error << endl;*/

	return error < 1e-5;
}

// 关于旋转矩阵 和 欧拉角 和 旋转矢量 部分直接使用 OpenCV 或者 Eigen库计算
// 以下函数启用 实际上 整个计算过程 只需要 旋转矩阵,
// 旋转矢量 是轴角的一种表示,用于控制机械臂
// 其单位向量为 旋转轴,其模长为旋转角


/*—————————————————————————————————————————————————————————
验证 https://www.andre-gaschler.com/rotationconverter/
从欧拉角计算旋转矢量,UR控制用
rpy 弧度
旋转向量即轴角 模长表角度 单位向量 为轴
—————————————————————————————————————————————————————————*/
Eigen::Vector3d get_rotation_vector_from_rpy(Eigen::Vector3d rpy)
{
	double alpha = rpy[2];
	double beta = rpy[1];
	double gamma = rpy[0];

	double ca = cos(alpha);
	double cb = cos(beta);
	double cg = cos(gamma);

	double sa = sin(alpha);
	double sb = sin(beta);
	double sg = sin(gamma);

	double r11 = ca * cb;
	double r12 = ca * sb * sg - sa * cg;
	double r13 = ca * sb * cg + sa * sg;
	
	double r21 = sa * cb;
	double r22 = sa * sb * sg + ca * cg;
	double r23 = sa * sb * cg - ca * sg;

	double r31 = -sb;
	double r32 = cb * sg;
	double r33 = cb * cg;

	double theta = acos((r11 + r22 + r33 - 1) / 2);
	double sth = sin(theta);

	double kx = (r32 - r23) / (2 * sth);
	double ky = (r13 - r31) / (2 * sth);
	double kz = (r21 - r12) / (2 * sth);

	Eigen::Vector3d rv{ theta * kx,theta * ky,theta * kz };
	return rv;
}

/*—————————————————————————————————————————————————————————
从旋转矢量计算欧拉角,UR控制用
rpy 弧度 
—————————————————————————————————————————————————————————*/
Eigen::Vector3d get_rpy_form_rotation_vector(Eigen::Vector3d rv)
{
	double theta = rv.norm();
	double kx = rv[0] / theta;
	double ky = rv[1] / theta;
	double kz = rv[2] / theta;

	double cth = cos(theta);
	double sth = sin(theta);
	double vth = 1 - cos(theta);

	double r11 = kx * kx * vth + cth;
	double r12 = kx * ky * vth - kz * sth;
	double r13 = kx * kz * vth + ky * sth;
	
	double r21 = kx * ky * vth + kz * sth;
	double r22 = ky * ky * vth + cth;
	double r23 = ky * kz * vth - kx * sth;

	double r31 = kx * kz * vth - ky * sth;
	double r32 = ky * kz * vth + kx * sth;
	double r33 = kz * kz * vth + cth;

	double alpha = 0, gamma = 0;

	double beta = atan2(-r31, sqrt(r11 * r11 + r21 * r21));

	double corner_case = 89.99 / 57.2958;// 临界
	if (beta > corner_case)
	{
		beta = corner_case;
		alpha = 0;
		gamma = atan2(r12, r22);
	}
	else if (beta < -corner_case)
	{
		beta = -corner_case;
		alpha = 0;
		gamma = -atan2(r12, r22);
	}
	else
	{
		double cb = cos(beta);
		alpha = atan2(r21 / cb, r11 / cb);
		gamma = atan2(r32 / cb, r33 / cb);
	}

	Eigen::Vector3d rpy{ gamma,beta,alpha };
	return rpy;
}


/*—————————————————————————————————————————————————————————
从旋转矩阵计算欧拉角(roll,pitch,yaw) rpy
旋转顺序 ZYX
返回顺序 vector3d (x,y,z)
—————————————————————————————————————————————————————————*/
Eigen::Vector3d get_rpy_from_rotation_matrix(Eigen::Matrix3d &R)
{
	assert(is_rotation_matrix(R));

	double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
	bool singular = sy < 1e-5;

	double x, y, z;
	if (!singular)
	{
		x = atan2(R(2, 1), R(2, 2));
		y = atan2(-R(2, 0), sy);
		z = atan2(R(1, 0), R(0, 0));
	}
	else
	{
		x = atan2(-R(1, 2), R(1, 1));
		y = atan2(-R(2, 0), sy);
		z = 0;
	}
	Eigen::Vector3d rpy{x,y,z};
	return rpy;
}

// 输入两个向量 计算余弦值
double vector3d_to_angle(Eigen::Vector3d p0, Eigen::Vector3d p1,Eigen::Vector3d p2)
{
	double angle = 0;
	double cos_theta = 0;
	Eigen::Vector3d start_v = p1 - p0;
	Eigen::Vector3d end_v = p2 - p0;
	cos_theta = start_v.dot(end_v)/(start_v.norm()*end_v.norm());
	cout << "cos_theta: "<<cos_theta << endl;

	return cos_theta;
}

// 从5个点中获取 水平向量
Eigen::Vector3d get_nx_from_5points(vector<Eigen::Vector3d> &bigger_center)
{
	vector<Eigen::Vector3d> center_3p;
	Eigen::Vector3d nx_;

	if (bigger_center.size() != 5)
		return nx_;

	
	//  起始点 只需要 遍历三个点即可 ，三个点 必然包含在一条直线上的三个点之一
	// cos 值可能为-1
	for (int i = 0; i < 3; i++)
	{
		Eigen::Vector3d p0 = bigger_center[i];

		Eigen::Vector3d p1;
		Eigen::Vector3d p2;
		double cos_theta = 0;
		for (int j = 0; j < 5; j++) // 向量1 终点
		{
			if (j == i) // 跳过自身
				continue;
			p1 = bigger_center[j];

			for (int k = 0; k < 5; k++) // 向量2 的终点
			{
				if (k == i || k == j)
					continue;

				p2 = bigger_center[k];


				cout << i << "-->" << j << ", " << i << "-->" << k << endl;
				//cout << p0 << endl << p1 << endl << p2 << endl;

				cos_theta = vector3d_to_angle(p0, p1, p2);

				if (abs(cos_theta) > 0.95)
				{
					center_3p.push_back(p0);
					center_3p.push_back(p1);
					center_3p.push_back(p2);

					nx_ = (p1 - p0) / (p1 - p0).norm();
					if (nx_[1] > 0)
						nx_ = -nx_;
					return nx_;
				}
			}

		}
	}
}

/*—————————————————————————————————————————————————————————
主要算法代码:
处理点云数据,返回6D位姿
—————————————————————————————————————————————————————————*/
void process_pcd()
{
	if (!g_pcd_ptr->HasPoints())
		return;
	
	cout << "[Debug Info:] load the points: "<<g_pcd_ptr->points_.size() << endl;
	auto down_pcd = g_pcd_ptr->UniformDownSample(5);
	cout << "[Debug Info:] down pcd points: " << down_pcd->points_.size() << endl;
	//visualization::DrawGeometries({ down_pcd });

	//utility::VerbosityContextManager cm(utility::VerbosityLevel::Debug);
	vector<int> labels = down_pcd->ClusterDBSCAN(3, 5, true);
	// max_label+1
	int max_label = *std::max_element(labels.begin(), labels.end()) + 1;
	cout << "[Debug Info:] point cloud has " << max_label << " clusters" << endl;
		
	// 提取每一类的index
	cout << "[Debug Info:] split the cluster" << endl;
	vector<vector<size_t> > label_class;
	vector<size_t> temp_class;

	// 有几类遍历几遍,第一遍提取索引为0的位置,2遍提取1,...
	for (int i = 0; i < max_label; i++) 
	{
		for (int j = 0; j < labels.size(); j++) 
		{
			if (labels[j] == i)
			{
				temp_class.push_back(j);
			}
		}
		label_class.push_back(temp_class);
		temp_class.clear();
	}

	cout << "[Debug Info:] filter by num point" << endl;
	// 根据点数 过滤 提取满足要求的索引 和点云
	vector<vector<size_t> > obj_idx; 
	vector<shared_ptr<geometry::PointCloud> > obj_pcd;
	for (int i = 0; i < max_label; i++)
	{
		// 点数满足要求
		if (label_class[i].size() > 650 && label_class[i].size() < 1650)
		{
			obj_idx.push_back(label_class[i]);
			obj_pcd.push_back(down_pcd->SelectByIndex(label_class[i]));
		}
	}

	cout << "[Debug Info:] filter by area" << endl;
	// 根据包围框面积 过滤提取 充电口 表面 半圆
	// 在满足要求的点云中 提取包围框
	vector<shared_ptr<geometry::PointCloud> > face_pcd;
	for (size_t i = 0; i < obj_pcd.size(); i++)
	{
		auto obbx = obj_pcd[i]->GetOrientedBoundingBox();
		obbx.color_ = { 1,0,0 };
		vector<double> abc{ obbx.extent_[0],obbx.extent_[1],obbx.extent_[2] };
		sort(abc.begin(), abc.end());
		double rate = abc[2] / abc[1];
		if (rate > 0.9 && rate < 1.1 && abc[2]>50 && abc[2] < 70)
		{
			face_pcd.push_back(obj_pcd[i]);
		}
	}

	// 如果没有目标 则返回
	if (face_pcd.size()<1) 
	{
		cout << "[Debug Info:] object not found" << endl;
		return;
	}
		
	cout << "[Debug Info:] draw face" << endl;
	// 如果存在不止一个目标，使用第一个目标求，
	auto obbx1 = face_pcd[0]->GetOrientedBoundingBox();
	obbx1.color_ = { 1,0,0 };
	shared_ptr<geometry::OrientedBoundingBox>
		obx_ptr(new geometry::OrientedBoundingBox);
	*obx_ptr = obbx1;
	// 显示半圆表面以及包围框
	//visualization::DrawGeometries({ face_pcd[0], obx_ptr});

	cout << "[Debug Info:] fitting a plane" << endl;
	auto plane_para = face_pcd[0]->SegmentPlane(1, 3, 100);
	auto plane_coe = std::get<0>(plane_para);	// 平面方程系数
	auto plane_in = std::get<1>(plane_para);	// 内点索引
	cout.setf(ios::fixed);
	cout << setprecision(3); 
	cout<<"[Debug Info:] plane factor: " 
		<< plane_coe[0] << "x + " << plane_coe[1] << "y + "
		<< plane_coe[2] << "z + " << plane_coe[3] << " = 0" << endl;

	auto in_pcd = face_pcd[0]->SelectByIndex(plane_in);
	in_pcd->PaintUniformColor ({ 1,0,0 });

	auto out_pcd = face_pcd[0]->SelectByIndex(plane_in, true);
	out_pcd->PaintUniformColor({ 0.2,0.8,0.6 });

	// 显示平面拟合后的内外点
	//visualization::DrawGeometries({ in_pcd, out_pcd });

	cout << "[Debug Info:] crop point" << endl;
	geometry::OrientedBoundingBox crop_box;

	// 比原始包围框略大一点
	crop_box.center_ = obbx1.center_;
	crop_box.extent_ = {
		obbx1.extent_[0] * 1.0,
		obbx1.extent_[1] * 1.0,
		obbx1.extent_[2] * 5.5
	};

	auto crop_pcd = g_pcd_ptr->Crop(crop_box);
	crop_pcd->PaintUniformColor({ 0,0.6,0.2 });

	// 显示裁剪后的点云
	//visualization::DrawGeometries({ crop_pcd });
	//vis.AddGeometry(crop_pcd);

	auto sor_para = crop_pcd->RemoveStatisticalOutliers(60, 0.4);
	auto sor_pcd = std::get<0>(sor_para);
	auto sor_ind = std::get<1>(sor_para);

	auto sor_in_cloud = crop_pcd->SelectByIndex(sor_ind);
	sor_in_cloud->PaintUniformColor({ 1,0,0 });

	auto sor_out_cloud = crop_pcd->SelectByIndex(sor_ind, true);
	sor_out_cloud->PaintUniformColor({ 0,1,0 });

	// 显示去噪后的点
	//visualization::DrawGeometries({ sor_in_cloud,sor_out_cloud });

	// 计算裁剪的点到平面的距离
	cout << "[Debug Info:] distance" << endl;
	vector<size_t> circle_idx;
	shared_ptr<geometry::PointCloud> circles(new geometry::PointCloud());
	for (int i = 0; i < sor_in_cloud->points_.size(); i++)
	{

		double d = point_to_plane_distance(sor_in_cloud->points_[i], plane_coe);
		//if (i % 10 == 0) // for debug
		//	cout << d << endl;

		if (d > 3.5 && d < 7.5)
		{
			circle_idx.push_back(i);
		}
	}

	// 点数过少则结束
	if (circle_idx.size() < 1000)
	{
		return; 
	}
	// 显示七个小圆环
	cout << "[Debug Info:] show seven circle" << endl;
	circles = sor_in_cloud->SelectByIndex(circle_idx);
	//visualization::DrawGeometries({ circles }, "circles");

	// 对小环聚类
	auto labels_circle = circles->ClusterDBSCAN(3, 5); //3 5
	auto max_lc = *max_element(labels_circle.begin(), labels_circle.end());

	cout << "[Debug Info:] detect clusters:" << max_lc+1 << endl;
		
	
	// 分离出七个点云,有背景干扰时 会多于7个
	vector<shared_ptr<geometry::PointCloud> > vec_circle;
	for (int l = 0; l < max_lc+1; l++)
	{
		vector<size_t> temp_lc; // temp circle label
		for (int i = 0; i < labels_circle.size(); i++)
		{
			if (labels_circle[i] == l)
			{
				temp_lc.push_back(i);
			}
		}
		vec_circle.push_back(circles->SelectByIndex(temp_lc));
		temp_lc.clear();
	}

	// 分理出大圆和小圆
	vector<geometry::OrientedBoundingBox> vec_cir_box;  
	vector<Eigen::Vector3d> small_center;
	vector<Eigen::Vector3d> bigger_center;
		
	small_center.clear();
	bigger_center.clear();

	for (int i = 0; i < vec_circle.size(); i++)
	{
		int n = vec_circle[i]->points_.size();
		cout <<"points: " << n << endl;

		if(n<40) //噪声点  100
			continue;

		auto temp_box = vec_circle[i]->GetOrientedBoundingBox();
		vec_cir_box.push_back(temp_box);
			
		vector<double> abc{
			temp_box.extent_[0],
			temp_box.extent_[1],
			temp_box.extent_[2]
		};
			  
		sort(abc.begin(), abc.end());
		auto obx_area = abc[2] * abc[1];
		auto rate = abc[2] / abc[1];
		cout << "rate: " << rate << endl;
		//cout << "[Debug Info:] abc: " << abc[2]
		//	<< " " << abc[1] << " " << abc[0] << endl;
		//cout << "[Debug Info:] area: " << obx_area << endl;
			

		if (rate > 1.0 && rate < 1.35)
		{
			cout << obx_area << endl;
			if (obx_area > 20 && obx_area < 80) // 40
			{
				small_center.push_back(temp_box.center_);
			}
			else if (obx_area > 100 && obx_area < 200)//140
			{
				bigger_center.push_back(temp_box.center_);
			}
		}
	}

	cout << "[Debug Info:] small:" << small_center.size()
		<< " bigger: " << bigger_center.size() << endl;

	/*cout << "bigger center :" << endl;
	for (int i = 0; i < bigger_center.size(); i++)
	{
		cout << bigger_center[i] << endl;
	}*/

	
	Eigen::Vector3d nx_3p = get_nx_from_5points(bigger_center);
	// 根据三点(任取其二)计算水平向量 单位向量
	if (nx_3p.norm()<0.5) // 不是单位向量
	{
		return;
	}

		
	/*if ((small_center.size() != 2) && (bigger_center.size() != 5))
	{
		return;
	}*/

	cout << "[Debug Info:] draw line" << endl;
	vector<Eigen::Vector3d> line_points;
	line_points.push_back(mean_point(small_center)); 
	line_points.push_back(mean_point(bigger_center));

	vector<Eigen::Vector2i> lines{ {0,1} }; // 0 for small
		
	shared_ptr<geometry::LineSet> line_ptr(new geometry::LineSet());
	*line_ptr = geometry::LineSet(line_points, lines);

	line_ptr->PaintUniformColor({ 0,0,1 });
	//visualization::DrawGeometries({ circles,line_ptr });

	// 计算平面法向量,计算位姿
	// 目标姿态,单位矢量,列向量
	
	Eigen::Vector3d cp{ line_points[1] };	//平移	大圆中心点     // 目标中心点
	Eigen::Vector3d n1{ plane_coe[0],plane_coe[1],plane_coe[2] };  // 法向量 z
	Eigen::Vector3d n2{ line_points[0] - line_points[1] };		   // x方向
	n2 = n2 / n2.norm(); // 计算获得
	Eigen::Vector3d n3 = n1.cross(n2);							   // y方向


	//-----------------
	// n1 应该需要单位化
	//-----------------

	// n2可能不垂直于n1,使用叉积重新计算一下
	n2 = n3.cross(n1); 

	// 旋转矩阵,即由目标姿态构成的矩阵 old
	/*Eigen::Matrix3d obj_rotate_matrix{ 
		{n2[0],n3[0],n1[0]},
		{n2[1],n3[1],n1[1]},
		{n2[2],n3[2],n1[2]},
	};*/

	// 2022-07-18 针对机械臂末端 xy方向调整
	Eigen::Vector3d ny_3p = nx_3p.cross(n1);
	if (ny_3p[0] < 0) // 相对于 相机坐标系
	{
		ny_3p = -ny_3p;
	}

	// 旋转矩阵,即由目标姿态构成的矩阵
	Eigen::Matrix3d obj_rotate_matrix{
		{nx_3p[0],ny_3p[0],n1[0]},
		{nx_3p[1],ny_3p[1],n1[1]},
		{nx_3p[2],ny_3p[2],n1[2]},
	};



	cout << "center point: \n" << cp.transpose() << endl;
	cout << "rotate matrix:\n" << obj_rotate_matrix << endl;


	// 相机坐标系下沿主轴方向的单位矢量,可构成单位矩阵
	Eigen::Vector3d O{ 0,0,0 };		// 原点
	Eigen::Vector3d nx{ 1,0,0 };
	Eigen::Vector3d ny{ 0,1,0 };
	Eigen::Vector3d nz{ 0,0,1 };

	// 标定的结果
	Eigen::Matrix4d base_H_cam{
		{-5.2938610e-03, -9.999836e-01,	-2.025038e-03, 1.0374186e+02},
		{1.17532312e-02, 1.9627213e-03,	-9.9992871e-01, 5.0691424e+02},
		{9.99916493e-01, -5.3174346e-03, 1.1742502e-02, 5.0810568e+02},
		{0., 0., 0., 1.}
	};

	Eigen::Matrix4d cam_H_base = base_H_cam.inverse();
	/*Eigen::Matrix4d cam_H_obj{
		{n2[0],n3[0],n1[0],cp[0]},
		{n2[1],n3[1],n1[1],cp[1]},
		{n2[2],n3[2],n1[2],cp[2]},
		{0,0,0,1}
	};*/

	Eigen::Matrix4d cam_H_obj{
		{nx_3p[0],ny_3p[0],n1[0],cp[0]},
		{nx_3p[1],ny_3p[1],n1[1],cp[1]},
		{nx_3p[2],ny_3p[2],n1[2],cp[2]},
		{0,0,0,1}
	};

	// 计算路径点
	auto path_cp = cp - 110 * n1;// 后退 110mm
	Eigen::Matrix4d cam_H_path{
		{nx_3p[0],ny_3p[0],n1[0],path_cp[0]},
		{nx_3p[1],ny_3p[1],n1[1],path_cp[1]},
		{nx_3p[2],ny_3p[2],n1[2],path_cp[2]},
		{0,0,0,1}
	};
	Eigen::Matrix4d re_base_H_path = base_H_cam * cam_H_path;

	// 姿态部分其实和目标一样 只需计算平移分量即可
	//Eigen::Matrix3d path_rmat = re_base_H_path.block(0, 0, 3, 3);
	//Eigen::AngleAxisd path_rvec{ path_rmat }; // 构造函数直接转换
	////re_rvec.fromRotationMatrix(re_rmat);
	//Eigen::Vector3d p_rvec = path_rvec.angle() * path_rvec.axis(); 

	// 绕固定坐标系运动 左乘
	// 绝对位姿 没意义 仅用于绘图
	Eigen::Matrix4d ab_base_H_obj = cam_H_obj * base_H_cam;

	// 绕动坐标系运动 右乘
	// p_base * H_base2obj = p_obj
	// H_base2obj = inv(p_base) * p_obj 
	//            = inv(cam2base) * cam2obj
	// 相对位姿
	Eigen::Matrix4d re_base_H_obj = base_H_cam * cam_H_obj;

	cout << "cam2obj( obj pose ):\n" << cam_H_obj << endl;
	cout << "cam2base( base pose ):\n" << cam_H_base << endl;
	cout << "absolute base2obj:\n" << ab_base_H_obj << endl;
	cout << "relative base2obj:\n" << re_base_H_obj << endl;

	// or use cv::Rodrigues
	Eigen::Matrix3d re_rmat = re_base_H_obj.block(0, 0, 3, 3);
	Eigen::AngleAxisd re_rvec{ re_rmat }; // 构造函数直接转换
	//re_rvec.fromRotationMatrix(re_rmat);
	
	Eigen::Vector3d rvec = re_rvec.angle() * re_rvec.axis();
	/*cout << "Eigen relative rotation vector:\n";
	cout << rvec << endl;*/

	vector<double> move_path{
		re_base_H_path(0,3)/1000,
		re_base_H_path(1,3)/1000,
		re_base_H_path(2,3)/1000,
		rvec[0],
		rvec[1],
		rvec[2]
	};

	vector<double> move_pose{ 
		re_base_H_obj(0,3)/1000,		// x
		re_base_H_obj(1,3)/1000,		// y
		re_base_H_obj(2,3)/1000,		// z
		rvec[0],				// rx
		rvec[1],				// ry
		rvec[2]					// rz
	};

	
	// 全局变量
	g_pose.clear();
	g_pose.push_back(move_path);
	g_pose.push_back(move_pose);

	// 相对于相机坐标而言 变化范围最大的是 z 
	// 相对于机械臂基坐标而言 变化范围最大的是 y
	cout << "The relative pose are: \n";
	for (int i = 0; i < g_pose.size(); i++)
	{
		for (int j = 0; j < g_pose[i].size(); j++)
		{
			cout << g_pose[i][j] << " ";
		}
		cout << endl;
	}

	vector<Eigen::Vector3d> frame_points;
	vector<Eigen::Vector2i> frame_lines{ {0,1},{0,2},{0,3} };
		
	// 用于画线的点 lineset
	frame_points.clear();
	frame_points.push_back(O);
	frame_points.push_back(n1); //z,b 平面法向量  
	frame_points.push_back(n2); //x,r 计算的方向  
	frame_points.push_back(n3);	//y,g n1×n2

	shared_ptr<geometry::LineSet> frame_lineset(new geometry::LineSet());
	*frame_lineset = geometry::LineSet(frame_points, frame_lines);

	//以line_points[1]为起点
	
	frame_lineset->Translate(cp);// 移动到中心
	frame_lineset->Scale(30, cp);
		
	// 绘制线 使用mesh代替
	//visualization::DrawGeometries({ face_pcd[0],circles,frame_lineset });


	auto obj_frame = geometry::TriangleMesh::CreateCoordinateFrame();

	obj_frame->Translate(cp);
	obj_frame->Rotate(obj_rotate_matrix, cp);
		
	obj_frame->Scale(100, cp);

	// 加载三维网格图
	string cam_path = "F:/DataSet/Camera3d_right.PLY";
	string ur5_path = "F:/DataSet/UR5_right.PLY";
	string port_path = "F:/DataSet/port_right.ply";

	shared_ptr<geometry::TriangleMesh> ur5_mesh(new geometry::TriangleMesh());
	shared_ptr<geometry::TriangleMesh> cam_mesh(new geometry::TriangleMesh());
	shared_ptr<geometry::TriangleMesh> port_mesh(new geometry::TriangleMesh());
	
	io::ReadTriangleMesh(ur5_path, *ur5_mesh);
	io::ReadTriangleMesh(cam_path, *cam_mesh);
	io::ReadTriangleMesh(port_path, *port_mesh);

	// 相机在原点
	ur5_mesh->Transform(cam_H_base);
	port_mesh->Transform(ab_base_H_obj);

	cam_mesh->ComputeVertexNormals();
	ur5_mesh->ComputeVertexNormals(); // 显示轮廓
	port_mesh->ComputeVertexNormals();

	// 坐标轴
	auto ur5_frame = geometry::TriangleMesh::CreateCoordinateFrame(100,Eigen::Vector3d(0,0,0));
	ur5_frame->Transform(cam_H_base);

	auto path_frame = geometry::TriangleMesh::CreateCoordinateFrame(100, Eigen::Vector3d(0, 0, 0));
	path_frame->Transform(cam_H_path);


	// 实时显示 会造成阻塞 无法操作窗口
	/*vis.ClearGeometries();
	vis.AddGeometry(circles);
	vis.AddGeometry(face_pcd[0]);
	vis.AddGeometry(frame_mesh);
	vis.AddGeometry(line_ptr);
	vis.UpdateGeometry();
	vis.PollEvents();
	vis.UpdateRender();*/

	visualization::DrawGeometries({g_pcd_ptr, face_pcd[0],circles,obj_frame,ur5_mesh,cam_mesh,path_frame });

	//string ensure="";
	cout << "please input 'yes' or 'no'" << endl;
	cin >> g_ensure;
	//g_ensure = ""; //for not move
	

	//——————————————————————————————————————————————————————————————————
	//                         above  success 
	//——————————————————————————————————————————————————————————————————

	// 添加10帧方差不大的图进行处理
	// 
	// 直线转换为 6D 位姿
	// 机械臂 y负轴为向前

}

/*—————————————————————————————————————————————————————————
* main
* 相机采集,点云处理逻辑
—————————————————————————————————————————————————————————*/
int main00(int argc, char* argv[])
{
	
	// 直接用auto 减少代码量
	auto pcd_ptr = std::make_shared<geometry::PointCloud>();
	
	//vis.CreateVisualizerWindow("Detect 6D pose by dorob@qq.com",800,600);

	RTDEControlInterface rtde_control("192.168.1.101");
	RTDEReceiveInterface rtde_receive("192.168.1.101");
	auto get_pose = rtde_receive.getActualTCPPose();

	for (int i = 0; i < get_pose.size(); i++)
	{
		cout << get_pose[i] << " ";
	}


	//读取点云，后续改为3D相机读取 该点云为白色,Open3d显示时按字母上的数字3渲染
	int ret = 0;
	int num = 0;
	g_Device_list.clear();
	
	ret = Discovery_Start(DiscoveryCallfunc); // 开始寻找设备
	if (0 == ret)
	{
		Scanner* pScanner = CreateScannerImp();
		g_pScanner = pScanner;
		int cmd = -1;

		string str_cmd;

		while (true)
		{
			// 如果没有设备
			if (g_Device_list.size() == 0)
			{
				cout << "not find device" << endl;
				Sleep(1000);
				continue;
			}

			// 有设备
			for (auto &device:g_Device_list)
			{
				cout << "ID:" << device.deviceid
					<< " IP:" << device.ip
					<< " mac:" << device.mac
					<< " status:" << device.status
					<< " sdk:" << device.sdkversion << endl;
			}

			// 连接第1个设备,如果有多个IP(UR5+3D相机)则需要指定IP
			cmd = 1;
			if (0 == strcmp("ready", g_Device_list[cmd-1].status))
			{
				strncpy_s(g_Server_IP, g_Device_list[cmd - 1].ip, strlen(g_Device_list[cmd - 1].ip) + 1);
				strncpy_s(g_Mac_address, g_Device_list[cmd - 1].mac, strlen(g_Device_list[cmd - 1].mac) + 1);
				pScanner->Close(); // 关闭扫描设备

				cout << "open scanner " << g_Server_IP << endl;

				// 指定ip
				//if (0 != pScanner->Open(g_Server_IP, ConnectedCallfunc))
				if (0 != pScanner->Open("192.168.1.67", ConnectedCallfunc))
				{
					cout << "open scanner failed,try again!" << endl;
					Sleep(1000);
					continue;
				}
				pScanner->GetCoreDumpFile("./");
				break;
			}
			//如果设备没有准备好 即busy, 则重试
			else
			{
				cout << "the device busy,please select again!" << endl;
				continue;
			}	
		} // detect and connect

		printf_s("\n Please select the capture mode :\n");
		printf_s("\t0: Exit \n");
		printf_s("\t1: Automatic Capture \n");
		printf_s("\t2: Manual Capture ---> : ");
		DLP_STD_CIN_GET(cmd);
		if (cmd == 1)
		{
			printf_s("\n Please input the capture times ---> : ");
			std::cin >> num;
		}
		int count = 0;

		do //capture and process
		{
			if (!g_bConnected)
			{
				printf_s("not connect device , please wait! \n");
				Sleep(1000);
				continue;
			}
			// 是否正在 上传 coredump 文件
			if (pScanner->IsCoreDumpFileUploading())
			{
				printf_s("coredump file is uploading, please wait! \n");
				Sleep(1000);
			}
			else
			{
				if (g_bConnected)
				{
					switch (cmd)
					{
					case 1:
					{
						count++;
						if (count > num)
						{
							printf_s("\n Auto mode has ended \n\n");// end
							delete pScanner;
							system("pause");

							return 0;
						}

						printf_s("\n Auto Mode is running -- %d / %d --\n", count, num);
						pScanner->StartScan(StreamCallfunc);
						break;
					}
					case 2:
					{
						printf_s("\n Press any key to continue capture ~ \n");
						if(_getch()=='q')// 无缓冲获取键盘输入后继续
							cmd=0;
						pScanner->StartScan(StreamCallfunc);

						break;
					}

					default:
						printf_s("\n invalid menu select (cmd=%d)! \n", cmd);
						break;
					}
				}
				else // 没有连接
				{
					printf_s("\n the net disconnected, need reconnect! \n");
				}
			}
			process_pcd(); // my function
			if (g_ensure == "yes")
			{
				g_ensure = "";
				cout << "The robot is moving:" << endl;
				
				rtde_control.moveL(g_pose[0]);
				//Sleep(3000);
				
			}


		} while (cmd != 0); //capture and process

		delete pScanner;
	}

	//vis.DestroyVisualizerWindow();
	return 0;
}


/*—————————————————————————————————————————————————————————
* main
// 测试本地文件 2022-07-14 OK
// python 验证 坐标变换 见
// "D:\CodeTest\pythonProject\generate_frame.ipynb"
—————————————————————————————————————————————————————————*/

int main01()
{
	for (int i = 1; i < 8; i++)
	{
		string obj_path = "F:/DataSet/cam_0711/com_"+to_string(i)+".pcd";
		auto pcd_ptr = std::make_shared<geometry::PointCloud>();
		io::ReadPointCloud(obj_path, *pcd_ptr);
		g_pcd_ptr = pcd_ptr;
		process_pcd();
	}
    return 0;
	
}

int main02()
{
	Eigen::Vector3d p0{ 53,166,1128 };
	Eigen::Vector3d p1{ 53,181,1132 };
	Eigen::Vector3d p2{ 53,197,1135 };
	auto start_v = p1 - p0;
	auto end_v = p2 - p0;

	vector3d_to_angle(p0, p1, p2);
    return 0;
}
