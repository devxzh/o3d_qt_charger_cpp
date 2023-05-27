#include "my_detect.hpp"
MyDetect::MyDetect():
    src_pcd_ptr(new geometry::PointCloud),
    dst_pcd_ptr(new geometry::PointCloud),
    uni_pcd_ptr(new geometry::PointCloud),
    inner_pcd_ptr(new geometry::PointCloud),
    outer_pcd_ptr(new geometry::PointCloud)
{
}
MyDetect::~MyDetect()
{
}
/*———————————————————————————————————————————————————————————————————————————
// 从文件 或相机读取 点云
———————————————————————————————————————————————————————————————————————————*/
bool MyDetect::load_pcd_from_file_or_camera(bool is_camera,PcdPtr pcd,string file_name)
{
    bool is_open = false;
    if (is_camera)
    {
        src_pcd_ptr = pcd;
        is_open = (src_pcd_ptr->HasPoints()) ? true : false;
    }
    else
    {
        is_open = io::ReadPointCloud(file_name, *src_pcd_ptr);
    }
    return is_open;
}
/*———————————————————————————————————————————————————————————————————————————
// 降采样,及滤波
———————————————————————————————————————————————————————————————————————————*/
void MyDetect::uniform_down_filter_src(int k )
{
    uni_pcd_ptr = src_pcd_ptr->UniformDownSample(k);
    //auto sor = uni_pcd_ptr->RemoveRadiusOutliers(50, 10);
    auto sor = uni_pcd_ptr->RemoveStatisticalOutliers(10, 5.0);
    auto sor_index = std::get<1>(sor);
    auto sor_in_cloud = uni_pcd_ptr->SelectByIndex(sor_index);
    uni_pcd_ptr = sor_in_cloud;
}
/*———————————————————————————————————————————————————————————————————————————
// 整体聚类
———————————————————————————————————————————————————————————————————————————*/
void MyDetect::split_pcd_by_cluster_dbscan_src(double r , int min_points)
{
    // 得到每个点的标签, 类别 = 标签+1
    vector<int> labels = uni_pcd_ptr->ClusterDBSCAN(r, min_points, false);
    int max_label = *std::max_element(labels.begin(), labels.end()) + 1;
    //cout << "[Debug Info:] point cloud has " << max_label << " clusters" << endl;
    // 根据标签提取每一簇
    vector<size_t> one_cluster;
    for (int i=0;i<max_label;i++)
    {
        for (int j = 0; j < labels.size(); j++)
        {
            if (labels[j] == i) // 提取i类的索引
            {
                one_cluster.push_back(j);
            }
        }
        all_cluster.push_back(one_cluster);
        one_cluster.clear();
    }
}
/*———————————————————————————————————————————————————————————————————————————
// 根据聚类 点数 过滤
———————————————————————————————————————————————————————————————————————————*/
void MyDetect::get_outer_pcd_by_points_nums(int min_points, int max_points)
{
    for (int i = 0; i < all_cluster.size(); i++)
    {
        if (all_cluster[i].size() > min_points && all_cluster[i].size() < max_points)
        {
            obj_pcds.push_back(uni_pcd_ptr->SelectByIndex(all_cluster[i]));
        }
    }
}
/*———————————————————————————————————————————————————————————————————————————
// 根据聚类边长 或 面积 过滤
———————————————————————————————————————————————————————————————————————————*/
void MyDetect::get_outer_pcd_by_points_obbx(double min_area, double max_area)
{
    vector<shared_ptr<geometry::PointCloud> > face_pcd;
    for (size_t i = 0; i < obj_pcds.size(); i++)
    {
        auto obbx = obj_pcds[i]->GetOrientedBoundingBox();
        obbx.color_ = { 1,0,0 };
        vector<double> abc{ obbx.extent_[0],obbx.extent_[1],obbx.extent_[2] };
        sort(abc.begin(), abc.end());
        double rate = abc[2] / abc[1];
        if (rate > 0.9 && rate < 1.2 && abc[2]>50 && abc[2] < 70)
        {
            face_pcd.push_back(obj_pcds[i]);
        }
    }
    if (face_pcd.size() > 0)
    {
        outer_pcd_ptr = face_pcd[0]; // 用第一个求解，这里没有处理 没有目标的情况
        outer_pcd_ptr->PaintUniformColor({0.5,0.2,0.3});
        outer_pcd_obbx = outer_pcd_ptr->GetOrientedBoundingBox();
        outer_cp = outer_pcd_obbx.center_;	 // 这里中心点可能上下浮动
        //visualization::DrawGeometries({ out_pcd_ptr });
    }
}
/*———————————————————————————————————————————————————————————————————————————
// 根据 扩大外圈包围框 框选 内圈
———————————————————————————————————————————————————————————————————————————*/
void MyDetect::get_inner_pcd_by_expend_out_obbx(double kx, double ky, double kz)
{
    // 原始包围框略大
    OBBX expend_bbox;
    expend_bbox.center_ = outer_pcd_obbx.center_;
    expend_bbox.extent_ = {
        outer_pcd_obbx.extent_[0] * kx,
        outer_pcd_obbx.extent_[1] * ky,
        outer_pcd_obbx.extent_[2] * kz,
    };
    inner_pcd_ptr = src_pcd_ptr->Crop(expend_bbox);
    // 计算裁剪点到 外面 平面的距离 通过阈值过滤
    vector<size_t> inner_circle_index;
    PcdPtr inner_circles_ptr(new geometry::PointCloud());
    for (int i = 0; i < inner_pcd_ptr->points_.size(); i++)
    {
        double d = point_to_plane_distance(inner_pcd_ptr->points_[i], outer_plane_coef);
        if (d > 3.5 && d < 7.5)
        {
            inner_circle_index.push_back(i);
        }
    }
    // 点数过少则结束
    if (inner_circle_index.size() < 1000)
    {
        return;
    }
    inner_circles_ptr = inner_pcd_ptr->SelectByIndex(inner_circle_index);
    inner_pcd_ptr = inner_circles_ptr; // 可只用一个变量
}
/*———————————————————————————————————————————————————————————————————————————
// 拟合外面的平面,用于过滤后续内插口之外的点(多裁剪的点)
———————————————————————————————————————————————————————————————————————————*/
void MyDetect::fit_outer_plane()
{
    if(!outer_pcd_ptr->HasPoints()) // 没有点
        return;
    auto plane_para = outer_pcd_ptr->SegmentPlane(1, 3, 100);
    auto plane_coef = std::get<0>(plane_para);	// 平面方程系数
    auto plane_index = std::get<1>(plane_para);	// 内点索引
    outer_plane_coef = plane_coef;
// 显示平面内外的点
    cout.setf(ios::fixed);
    //cout << setprecision(3);
//    cout << "Plane: "
//        << plane_coef[0] << "x + " << plane_coef[1] << "y + "
//        << plane_coef[2] << "z + " << plane_coef[3] << " = 0" << endl;
//
// 	auto in_pcd = outer_pcd_ptr->SelectByIndex(plane_index);
// 	in_pcd->PaintUniformColor({ 1,0,0 });
//
// 	auto out_pcd = outer_pcd_ptr->SelectByIndex(plane_index, true);
// 	out_pcd->PaintUniformColor({ 0.2,0.8,0.6 });
}
/*———————————————————————————————————————————————————————————————————————————
// 获取中心点 ,可将内部七个圆环聚类为一个簇,暂不需要
———————————————————————————————————————————————————————————————————————————*/
void MyDetect::get_inner_pcd_by_cluster(double r, int min_points)
{
    /*auto sor = inner_pcd_ptr->RemoveStatisticalOutliers(15, 1.5);
    auto sor_index = std::get<1>(sor);
    inner_pcd_ptr = inner_pcd_ptr->SelectByIndex(sor_index);*/
    auto sor = inner_pcd_ptr->RemoveRadiusOutliers(30, 2.0);
    auto sor_index = std::get<1>(sor);
    inner_pcd_ptr = inner_pcd_ptr->SelectByIndex(sor_index);
    // 聚类求 包围框中心
    // 对小环聚类
    auto inner_labels = inner_pcd_ptr->ClusterDBSCAN(1.5, 5);
    auto max_inner_label = *max_element(inner_labels.begin(), inner_labels.end());
    max_inner_label += 1;
    //cout << "[Debug Info:] detect clusters:" << max_inner_label << endl;
    // 分离出七个点云,有背景干扰时 会多于7个
    vector<shared_ptr<geometry::PointCloud> > vec_circle;
    for (int l = 0; l < max_inner_label; l++)
    {
        vector<size_t> temp_label; // temp circle label
        for (int i = 0; i < inner_labels.size(); i++)
        {
            if (inner_labels[i] == l)
            {
                temp_label.push_back(i);
            }
        }
        vec_circle.push_back(inner_pcd_ptr->SelectByIndex(temp_label));
        temp_label.clear();
    }
    // 分理出大圆和小圆
    vector<OBBX> vec_cir_box;
    vector<Eigen::Vector3d> bigger_centers;
    bigger_centers.clear();
    for (int i = 0; i < vec_circle.size(); i++)
    {
        size_t n = vec_circle[i]->points_.size();
        if (n < 100) //噪声点
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
        //cout << "[Debug Info:] abc: " << abc[2]
        //	<< " " << abc[1] << " " << abc[0] << endl;
        //cout << "[Debug Info:] area: " << obx_area << endl;
        if (rate > 0.9 && rate < 1.2)
        {
            //cout << obx_area << endl;
            if (obx_area > 100 && obx_area < 200) // 40
            {
                bigger_centers.push_back(temp_box.center_);
            }
        }
    }
    //cout << "[Debug Info:] bigger: " << bigger_centers.size() << endl;
    if (bigger_centers.size() <1)  // 不必完全得到七个圈
    {
        return;
    }
    // 根据外环中心点距离 找到内环中心点
    double min_distance = 1000;
    Eigen::Vector3d temp_p;
    auto temp_cp = outer_pcd_obbx.center_;
    for (auto& p : bigger_centers)
    {
        auto temp_dis = (temp_cp - p).norm();
        if (temp_dis < min_distance)
        {
            min_distance = temp_dis;
            temp_p = p;
        }
    }
    inner_cp7 = temp_p;
    //visualization::DrawGeometries({ inner_pcd_ptr });
}
/*———————————————————————————————————————————————————————————————————————————
// 对内部七个圆环聚类, 这一部分没用到,见 get_inner_pcd_by_cluster
———————————————————————————————————————————————————————————————————————————*/
void MyDetect::get_inner_direction_by_cluster(double r, double min_points)
{
    // 对小环聚类
    auto inner_labels = inner_pcd_ptr->ClusterDBSCAN(1.5, 5);
    auto max_inner_label = *max_element(inner_labels.begin(), inner_labels.end());
    max_inner_label += 1;
    //cout << "[Debug Info:] detect clusters:" << max_inner_label << endl;
    // 分离出七个点云,有背景干扰时 会多于7个
    vector<shared_ptr<geometry::PointCloud> > vec_circle;
    for (int l = 0; l < max_inner_label; l++)
    {
        vector<size_t> temp_label; // temp circle label
        for (int i = 0; i < inner_labels.size(); i++)
        {
            if (inner_labels[i] == l)
            {
                temp_label.push_back(i);
            }
        }
        vec_circle.push_back(inner_pcd_ptr->SelectByIndex(temp_label));
        temp_label.clear();
    }
    // 分理出大圆和小圆
    vector<OBBX> vec_cir_box;
    vector<Eigen::Vector3d> small_centers;
    vector<Eigen::Vector3d> bigger_centers;
    small_centers.clear();
    bigger_centers.clear();
    for (int i = 0; i < vec_circle.size(); i++)
    {
        size_t n = vec_circle[i]->points_.size();
        if (n < 100) //噪声点
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
        //cout << "[Debug Info:] abc: " << abc[2]
        //	<< " " << abc[1] << " " << abc[0] << endl;
        //cout << "[Debug Info:] area: " << obx_area << endl;
        if (rate > 0.9 && rate < 1.2)
        {
            //cout << obx_area << endl;
            if (obx_area > 20 && obx_area < 80) // 40
            {
                small_centers.push_back(temp_box.center_);
            }
            else if (obx_area > 100 && obx_area < 200)//140
            {
                bigger_centers.push_back(temp_box.center_);
            }
        }
    }
    //cout << "[Debug Info:] small:" << small_centers.size()
    //	<< " bigger: " << bigger_centers.size() << endl;
    if ((small_centers.size() != 2) && (bigger_centers.size() != 5))
    {
        return;
    }
    // 内点的竖向量
    inner_cp2 = mean_point(small_centers);
    inner_nx = mean_point(bigger_centers) - inner_cp2;
    inner_ny = small_centers[1] - small_centers[0];
}
/*———————————————————————————————————————————————————————————————————————————
// 以点为中心迭代 获取包围框内的点,并重新计算包围框
———————————————————————————————————————————————————————————————————————————*/
void MyDetect::get_outer_direction_by_iterate_abbx(double len)
{
    double min_b = 5000.0;
    Eigen::Vector3d min_bound(-len, -len, -len);
    auto min_box = geometry::OrientedBoundingBox();
    for (auto& p : outer_pcd_ptr->points_)
    {
        auto& center = p;
        auto crop_box = geometry::AxisAlignedBoundingBox(min_bound, -min_bound);
        crop_box.Translate(center);
        crop_box.color_ = Eigen::Vector3d(0, 0.5, 0);
        auto crop_pcd = outer_pcd_ptr->Crop(crop_box);
        auto crop_pcd_bbox = crop_pcd->GetOrientedBoundingBox();
        auto abc = crop_pcd_bbox.extent_;
        vector<double> vec_abc{ abc[0], abc[1], abc[2] };
        sort(vec_abc.begin(), vec_abc.end());
        double temp_b = vec_abc[1]; // 处于中间长度的边,即宽度
        if (temp_b < min_b)
        {
            min_b = temp_b;
            min_box = crop_pcd_bbox;
            //min_box.color_ = Eigen::Vector3d(0.6, 0.6, 0);
        }
    }
    // 最长边为水平向量, 对应点云坐标系 ny
    //index: 0 origin, 1 -> x , 2 -> y , 3 -> z
    auto bbox_points = min_box.GetBoxPoints();
    outer_roi_obbx = min_box;
    double max_len = 0;
    Eigen::Vector3d temp_outer_ny;
    for (int i = 1; i < 4; i++)
    {
        auto temp_vec = bbox_points[i] - bbox_points[0];
        double temp_len = temp_vec.norm();
        if (temp_len > max_len)
        {
            max_len = temp_len;
            temp_outer_ny = temp_vec;
        }
    }
    outer_ny = temp_outer_ny / temp_outer_ny.norm(); // 单位化
}
/*———————————————————————————————————————————————————————————————————————————
 根据向量和中心点 计算最终姿态
 Note: 相机点云坐标系 x+(↑) y+(→),z+(向前)
       充电目标坐标系 x+(↑) y+(→),z+(向前)
       UR5机械臂末端	x+(←) y+(↑) z+(向前)
———————————————————————————————————————————————————————————————————————————*/
void MyDetect::caculate_pose()
{
    // 最后的中心点应结合 roi_obbx 中心的 y 和 outer_obbx 中心的 x
    // 用向量平移 计算,
    n0 = inner_cp7;		//outer_cp;
    ny = outer_ny;
    nz = Eigen::Vector3d
    {
        outer_plane_coef[0],
        outer_plane_coef[1],
        outer_plane_coef[2]
    };
    nx = ny.cross(nz); // 叉乘求法向量
    ny = nx.cross(nz); // ny 可能不垂直 nz
    // 单位化
    nx /= nx.norm();
    ny /= ny.norm();
    nz /= nz.norm();
    nx = (nx[0] < 0) ? -nx : nx;
    ny = (ny[1] < 0) ? -ny : ny;
    nz = (nz[2] < 0) ? -nz : nz;
    // 识别的结果, 规定坐标系(论文采用) x+(↑) y+(→),z+(向前)
    cam2obj = Eigen::Matrix4d
    {
        {nx[0],ny[0],nz[0],n0[0]},
        {nx[1],ny[1],nz[1],n0[1]},
        {nx[2],ny[2],nz[2],n0[2]},
        {	 0,	   0,	 0,	   1}
    };
    //cout << "cam2obj:\n" << cam2obj << endl;
    // 充电枪实际运动姿态 在充电口前,
    // 上下可能有平移,用向量加减计算, 或平移矩阵
    Eigen::Matrix4d obj2tool
    {
        {1.,0.,0.,0.},
        {0.,1.,0.,0.},
        {0.,0.,1.,-10.}, // z方向平移
        {0.,0.,0.,1.}
    };
    // 标定的结果
    base2cam = Eigen::Matrix4d
    {
        {-5.2938610e-03, -9.999836e-01,	-2.025038e-03,  1.0374186e+02},
        {1.17532312e-02, 1.9627213e-03,	-9.9992871e-01, 5.0691424e+02},
        {9.99916493e-01, -5.3174346e-03, 1.1742502e-02, 5.0810568e+02},
        {0., 0., 0., 1.}
    };
    cam2base = base2cam.inverse();
    // 路径点
    path_cp = n0 - 110 * nz; // 路径点
    cam2path = Eigen::Matrix4d
    {
        {nx[0],ny[0],nz[0],path_cp[0]},
        {nx[1],ny[1],nz[1],path_cp[1]},
        {nx[2],ny[2],nz[2],path_cp[2]},
        {	 0,	   0,	 0,			1}
    };
    Eigen::Matrix4d re_base2path = base2cam * cam2path;
    // 绝对位姿 没意义 仅用于绘图
    // 绕固定坐标系运动 左乘
    ab_base2obj = cam2obj * base2cam;
    // 相对位姿
    // 绕动坐标系运动 右乘
    // p_base * H_base2obj = p_obj
    // H_base2obj = inv(p_base) * p_obj
    //            = inv(cam2base) * cam2obj
    re_base2obj = base2cam * cam2obj;
    // 转换为 机械臂可接受的 旋转矢量
    // cv::Rodrigues
    Eigen::Matrix3d re_rotate_matrix = re_base2obj.block(0, 0, 3, 3);
    // 轴角 构造函数直接转换
    Eigen::AngleAxisd re_axis{ re_rotate_matrix };
    Eigen::Vector3d rvec = re_axis.angle() * re_axis.axis();
    vector<double> path_pose =
    {
        re_base2path(0,3)/1000,		// x
        re_base2path(1,3)/1000,		// y
        re_base2path(2,3)/1000,		// z
        rvec[0],	// rx
        rvec[1],	// ry
        rvec[2]		// rz
    };
    vector<double> obj_pose =
    {
        re_base2obj(0,3)/1000-nz[0]/1000*30, // 减少进给距离
        re_base2obj(1,3)/1000-nz[1]/1000*30,
        re_base2obj(2,3)/1000-nz[2]/1000*30,
        rvec[0],
        rvec[1],
        rvec[2]
    };
    ur_move_pose.clear();
    ur_move_pose.push_back(path_pose);
    ur_move_pose.push_back(obj_pose);
}
/*———————————————————————————————————————————————————————————————————————————
// 可视化指定图形
———————————————————————————————————————————————————————————————————————————*/
void MyDetect::vis_geometry()
{
    // 加载三维网格图
    string cam_path = "F:/DataSet/Camera3d_right.PLY";
    string ur5_path = "F:/DataSet/UR5_right.PLY";
    string port_path = "F:/DataSet/port_right.ply";
    shared_ptr<geometry::TriangleMesh> cam_mesh(new geometry::TriangleMesh());
    shared_ptr<geometry::TriangleMesh> ur5_mesh(new geometry::TriangleMesh());
    shared_ptr<geometry::TriangleMesh> port_mesh(new geometry::TriangleMesh());
    io::ReadTriangleMesh(cam_path, *cam_mesh);
    io::ReadTriangleMesh(ur5_path, *ur5_mesh);
    io::ReadTriangleMesh(port_path, *port_mesh);
    ur5_mesh->Transform(cam2base);
    port_mesh->Transform(cam2obj);
    // 使mesh显示更清晰
    cam_mesh->ComputeVertexNormals();
    ur5_mesh->ComputeVertexNormals();
    port_mesh->ComputeVertexNormals();
    // 坐标轴
    auto cam_frame = geometry::TriangleMesh::CreateCoordinateFrame(200, Eigen::Vector3d(0, 0, 0));
    //io::WriteTriangleMeshToPLY("frame.ply", *cam_frame,true,true,true,true,true,true);
    auto ur5_frame = geometry::TriangleMesh::CreateCoordinateFrame(200, Eigen::Vector3d(0, 0, 0));
    ur5_frame->Transform(cam2base);
    auto obj_frame = geometry::TriangleMesh::CreateCoordinateFrame(200, Eigen::Vector3d(0, 0, 0));
    obj_frame->Transform(cam2obj); // 使用位姿, 不可使用absolute_base2obj
    shared_ptr<OBBX> obj_obbx(new OBBX);
    *obj_obbx = outer_pcd_obbx;
    /*auto sphere = geometry::TriangleMesh::CreateSphere(2);
    sphere->Translate(inner_cp7);*/
    //visualization::DrawGeometries({ outer_pcd_ptr,obj_frame,inner_pcd_ptr },"detect");
    visualization::DrawGeometries({
            src_pcd_ptr,outer_pcd_ptr,obj_obbx,
            cam_frame,ur5_frame,obj_frame,
            cam_mesh,ur5_mesh/*,port_mesh*/
        });
}
/*———————————————————————————————————————————————————————————————————————————
 清空变量 避免对下一次循环造成数据污染
———————————————————————————————————————————————————————————————————————————*/
void MyDetect::clear_all()
{
    ur_move_pose.clear();
    all_cluster.clear();
    obj_cluster.clear();
    //src_pcd_ptr->Clear();
}
/*———————————————————————————————————————————————————————————————————————————
 添加所有处理函数 ,参数可调,顺序不可动
———————————————————————————————————————————————————————————————————————————*/
void MyDetect::pross_pcd(bool is_camera,PcdPtr pcd,string str)
{
    load_pcd_from_file_or_camera(is_camera, pcd, str);
    uniform_down_filter_src();
    split_pcd_by_cluster_dbscan_src();
    get_outer_pcd_by_points_nums(); // 先 点数
    get_outer_pcd_by_points_obbx();
    fit_outer_plane();
    get_inner_pcd_by_expend_out_obbx();
    get_inner_pcd_by_cluster();
    //get_inner_direction_by_cluster(1.5, 5);
    get_outer_direction_by_iterate_abbx();
    caculate_pose();
    pub_cam2obj=cam2obj; // api
    // vis_geometry(); 会阻塞
    //clear_all(); 调用ur后清除
}
/*———————————————————————————————————————————————————————————————————————————
计算点到平面的距离,点坐标,平面系数
———————————————————————————————————————————————————————————————————————————*/
double point_to_plane_distance(Eigen::Vector3d p, Eigen::Vector4d c)
{
    double da = std::abs(c[0] * p[0] + c[1] * p[1] + c[2] * p[2] + c[3]);
    double ds = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] + c[2]);
    return da / ds;
}
/*———————————————————————————————————————————————————————————————————————————
计算坐标平均值
———————————————————————————————————————————————————————————————————————————*/
Eigen::Vector3d mean_point(vector<Eigen::Vector3d> p)
{
    Eigen::Vector3d p_sum{ 0,0,0 };
    for (int i = 0; i < p.size(); i++)
    {
        p_sum += p[i];
    }
    return p_sum / p.size();
}
/*———————————————————————————————————————————————————————————————————————————
计算向量夹角,返回弧度
———————————————————————————————————————————————————————————————————————————*/
double get_angle(Eigen::Vector3d v1, Eigen::Vector3d v2)
{
    double cosAlpha = v1.dot(v2) / (v1.norm() * v2.norm());
    double rad = acos(cosAlpha) * 180 / 3.1415926;
    return rad;
}
/*———————————————————————————————————————————————————————————————————————————
判断是否为旋转矩阵 R*Rt=I
———————————————————————————————————————————————————————————————————————————*/
bool is_rotation_matrix(Eigen::Matrix3d R)
{
    auto I_mat = R * R.transpose();
    auto eye3 = Eigen::MatrixXd::Identity(3, 3);
    double error = (eye3 - I_mat).norm();
    /*cout << I_mat << endl;
    cout << error << endl;*/
    return error < 1e-5;
}
/*———————————————————————————————————————————————————————————————————————————
验证 https://www.andre-gaschler.com/rotationconverter/
从欧拉角计算旋转矢量,UR控制用
rpy 弧度
旋转向量即轴角 模长表角度 单位向量 为轴
———————————————————————————————————————————————————————————————————————————*/
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
/*———————————————————————————————————————————————————————————————————————————
从旋转矢量计算欧拉角,UR控制用
rpy 弧度
———————————————————————————————————————————————————————————————————————————*/
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
    //double r13 = kx * kz * vth + ky * sth;
    double r21 = kx * ky * vth + kz * sth;
    double r22 = ky * ky * vth + cth;
    //double r23 = ky * kz * vth - kx * sth;
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
/*———————————————————————————————————————————————————————————————————————————
从旋转矩阵计算欧拉角(roll,pitch,yaw) rpy
旋转顺序 ZYX
返回顺序 vector3d (x,y,z)
———————————————————————————————————————————————————————————————————————————*/
Eigen::Vector3d get_rpy_from_rotation_matrix(Eigen::Matrix3d& R)
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
    Eigen::Vector3d rpy{ x,y,z };
    return rpy;
}
