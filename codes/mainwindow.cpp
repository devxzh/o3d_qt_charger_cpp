#pragma once
#pragma comment(lib, "libscan_release.lib")
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>
#include "vtkAutoInit.h"
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
VTK_MODULE_INIT(vtkRenderingFreeType);
/*—————————————————————————————————————————————————————————
 全局变量
—————————————————————————————————————————————————————————*/
std::vector<DEVICE_INFO> g_device_list; // 设备列表
char g_mac_address[256] = {0};          // MAC地址
char g_server_ip[256] = "192.168.1.67"; // 相机IP
char g_device_id[256] = "COMD0511001";  // 相机ID
char g_net_mask[256] = "255.255.255.0"; // 子网掩码
bool g_is_connected = false;            // 连接与否
bool g_reconnect = false;               // 重连标志位
bool g_scan_status = true;              // 扫描完成状态
Scanner *g_pScanner = nullptr;          // 相机扫描仪
PcdPtr g_pcd_ptr(new geometry::PointCloud);
double g_src_x = 0; // 存储原始xyz
double g_src_y = 0; // 存储原始xyz
double g_src_z = 0; // 存储原始xyz
// 原IP 192.168.56.101  ur_rtde 1.5.1
// 是否使用设备
#define is_device false
#if is_device
ur_rtde::RTDEControlInterface rtde_control("192.168.1.101");
ur_rtde::RTDEReceiveInterface rtde_receive("192.168.1.101");
#endif
/*—————————————————————————————————————————————————————————
// 点击连接后，改变标志位，输出连接信息
—————————————————————————————————————————————————————————*/
void ConnectedCallfunc(bool connected)
{
    g_is_connected = connected;
    printf_s("ConnectedCallfunc (connected=%d)! \n", connected);
    return;
}
/*—————————————————————————————————————————————————————————
//回调函数, 获取设备信息
—————————————————————————————————————————————————————————*/
int DiscoveryCallfunc(DISCOVERY_TYPE_E type, char *dataAddr, int dataSize)
{
    char *start = NULL;
    char *end = NULL;
    DEVICE_INFO stDevice;
    memset(&stDevice, 0, sizeof(DEVICE_INFO));             // 设置设备信息为0
    if ((start = strstr(dataAddr, "deviceid:\"")) != NULL) // 查找第一次出现deviceid的位置
    {
        start += strlen("deviceid:\"");
        end = strstr(start, "\"");
        strncpy_s(stDevice.deviceid, start, end - start); // 获取设备ID
    }
    if ((start = strstr(dataAddr, "ipaddr:\"")) != NULL)
    {
        start += strlen("ipaddr:\"");
        end = strstr(start, "\"");
        strncpy_s(stDevice.ip, start, end - start); // 获取设备IP
    }
    if ((start = strstr(dataAddr, "hwadd:\"")) != NULL)
    {
        start += strlen("hwadd:\"");
        end = strstr(start, "\"");
        strncpy_s(stDevice.mac, start, end - start); // 获取设备MAC
    }
    if ((start = strstr(dataAddr, "status:\"")) != NULL)
    {
        start += strlen("status:\"");
        end = strstr(start, "\"");
        strncpy_s(stDevice.status, start, end - start); // 获取设备状态
    }
    if ((start = strstr(dataAddr, "sdkversion:\"")) != NULL)
    {
        start += strlen("sdkversion:\"");
        end = strstr(start, "\"");
        strncpy_s(stDevice.sdkversion, start, end - start); // 获取SDK版本
    }
    bool find = false;
    for (int i = 0; i < g_device_list.size(); i++)
    {
        if (0 == strcmp(stDevice.mac, g_device_list[i].mac)) // mac地址相同时
        {
            strncpy_s(g_device_list[i].ip, stDevice.ip, strlen(stDevice.ip) + 1);
            strncpy_s(g_device_list[i].status, stDevice.status, strlen(stDevice.status) + 1);
            strncpy_s(g_device_list[i].sdkversion, stDevice.sdkversion, strlen(stDevice.sdkversion) + 1);
            strncpy_s(g_device_list[i].deviceid, stDevice.deviceid, strlen(stDevice.deviceid) + 1);
            find = true;
            break;
        }
    }
    if (!find) // 如果没有重复设备，就添加
    {
        g_device_list.push_back(stDevice);
    }
    return 0;
}
/*—————————————————————————————————————————————————————————
获取点云
——————————————————————————————————————————————————————————*/
shared_ptr<geometry::PointCloud> get_pointCloud(char *pData, int dataSize)
{
    std::shared_ptr<geometry::PointCloud> pcd_ptr(new geometry::PointCloud);
    float *pPointCloud = (float *)pData;
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
int StreamCallfunc(STREAM_TYPE_E type, char *dataAddr, int dataSize)
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
    else if (type == STREAM_TYPE_POINT_CLOUD) // 1
    {
        g_pcd_ptr = get_pointCloud(dataAddr, dataSize);
        printf_s("[Camera Info:] STREAM_TYPE_POINT_CLOUD: dataSize=%d \n", dataSize);
    }
    return 0;
}
// 相机连接和检测
void camera3d_init(void)
{
    // 读取点云，后续改为3D相机读取 该点云为白色,Open3d显示时按字母上的数字3渲染
    g_device_list.clear();
    int ret = Discovery_Start(DiscoveryCallfunc); // 开始寻找设备
    if (0 == ret)
    {
        g_pScanner = CreateScannerImp();
        int cmd = 1;
        while (true)
        {
            // 如果没有设备
            if (g_device_list.size() == 0)
            {
                cout << "Not find device" << endl;
                Sleep(1000);
                continue;
            }
            // 有设备
            for (auto &device : g_device_list)
            {
                cout << "ID:" << device.deviceid
                     << " IP:" << device.ip
                     << " MAC:" << device.mac
                     << " status:" << device.status
                     << " SDK:" << device.sdkversion << endl;
            }
            // 连接第1个设备,如果有多个IP(UR5+3D相机)则需要指定IP
            if (0 == strcmp("ready", g_device_list[cmd - 1].status))
            {
                strncpy_s(g_server_ip, g_device_list[cmd - 1].ip, strlen(g_device_list[cmd - 1].ip) + 1);
                strncpy_s(g_mac_address, g_device_list[cmd - 1].mac, strlen(g_device_list[cmd - 1].mac) + 1);
                g_pScanner->Close(); // 关闭扫描设备
                cout << "open scanner " << g_server_ip << endl;
                // 指定ip
                // if (0 != pScanner->Open(g_Server_IP, ConnectedCallfunc))
                if (0 != g_pScanner->Open("192.168.1.67", ConnectedCallfunc))
                {
                    cout << "open scanner failed,try again!" << endl;
                    Sleep(1000);
                    continue;
                }
                g_pScanner->GetCoreDumpFile("./");
                break;
            }
            // 如果设备没有准备好 即busy, 则重试
            else
            {
                cout << "the device busy,please select again!" << endl;
                continue;
            }
        } // detect and connect
    }
}
void camera3d_capture(void)
{
    if (!g_is_connected)
    {
        printf_s("not connect device , please wait! \n");
        Sleep(1000);
        return;
    }
    // 是否正在 上传 coredump 文件
    if (g_pScanner->IsCoreDumpFileUploading())
    {
        printf_s("coredump file is uploading, please wait! \n");
        Sleep(1000);
    }
    else
    {
        g_pScanner->StartScan(StreamCallfunc);
        Sleep(1000);
    }
}
// ok
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow),
      timer(new QTimer()),
      g_renderer(vtkSmartPointer<vtkRenderer>::New()),
      g_renderWindow(vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New()),
      g_pcd(vtkSmartPointer<vtkPolyData>::New()),
      g_pcd_actor(vtkSmartPointer<vtkActor>::New()),
      g_ur5_actor(vtkSmartPointer<vtkActor>::New()),
      g_cam_actor(vtkSmartPointer<vtkActor>::New()),
      g_obj_frame_actor(vtkSmartPointer<vtkActor>::New()),
      g_vtk_obj_pose(vtkSmartPointer<vtkMatrix4x4>::New()),
      g_o3d_pcd(new geometry::PointCloud),
      my_index(1),
      text_info("程序开始运行!\n"),
      ur5_flag(false),
      frame_flag(false),
      cam_flag(false),
      manual_flag(true) // 初始为手动模式
{
    ui->setupUi(this);
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    // ur cam mesh
    g_ur5_actor = load_mesh("F:/DataSet/UR5_right.PLY");
    g_cam_actor = load_mesh("F:/DataSet/Camera3d_right.PLY");
    g_obj_frame_actor = load_mesh("F:/DataSet/frame.ply");
    double mdata[16] =
        {// base2cam
         -5.2938610e-03, -9.999836e-01, -2.025038e-03, 1.0374186e+02,
         1.17532312e-02, 1.9627213e-03, -9.9992871e-01, 5.0691424e+02,
         9.99916493e-01, -5.3174346e-03, 1.1742502e-02, 5.0810568e+02,
         0., 0., 0., 1.};
    vtkNew<vtkMatrix4x4> cam2base, base2cam;
    base2cam->DeepCopy(mdata);
    vtkMatrix4x4::Invert(base2cam, cam2base);
    g_ur5_actor->SetUserMatrix(cam2base);
    g_src_x = g_ur5_actor->GetPosition()[0];
    g_src_y = g_ur5_actor->GetPosition()[1];
    g_src_z = g_ur5_actor->GetPosition()[2];
#if is_device
    // init camera
    camera3d_init();
#endif
    // connect
    connect(timer, &QTimer::timeout, this, &MainWindow::timer_update);
    // render and window
    vtkNew<vtkNamedColors> colors;
    // 初始加载一个点云
    geometry::PointCloud o3d_pcd;
    string file_name = "F:/DataSet/cam_0711/com_1.pcd";
    io::ReadPointCloud(file_name, o3d_pcd);
    text_info += "load: " + file_name;
    ui->textEdit->setText(QString::fromStdString(text_info));
    g_pcd_actor = o3d_pcd_to_vtk_actor(o3d_pcd);
    g_renderer->AddActor(g_pcd_actor);
    g_renderer->SetBackground(colors->GetColor3d("GhostWhite").GetData());
    ui->openGLWidget->setRenderWindow(g_renderWindow);
    this->update_show();
}
MainWindow::~MainWindow()
{
    delete ui;
}
// 更新显示
void MainWindow::update_show(string append_str)
{
    ui->openGLWidget->update();
    g_renderWindow->AddRenderer(g_renderer);
    g_renderWindow->SetWindowName("RenderWindowNoUIFile");
    g_renderWindow->Render();
}
// 自动模式,从本地获取
void MainWindow::timer_update(void)
{
    g_renderer->RemoveActor(g_pcd_actor);
    string filename = "F:/DataSet/pcd_0730/class_255/com_" + to_string(my_index) + ".pcd";
    geometry::PointCloud pcd_ptr;
    io::ReadPointCloud(filename, pcd_ptr);
    g_pcd_actor = nullptr;
    g_pcd_actor = o3d_pcd_to_vtk_actor(pcd_ptr);
    g_pcd_actor->GetProperty()->SetColor(0.3, 0.5, 0.6); /*my_index/10.0,1-my_index/10.0,0.5*/
    g_renderer->AddActor(g_pcd_actor);
    ui->textEdit->append(QString::fromStdString(filename));
    g_o3d_pcd = make_shared<geometry::PointCloud>(pcd_ptr);
    // process pcd
    try
    {
        MyDetect *mydetect = new MyDetect;
        mydetect->pross_pcd(true, g_o3d_pcd, "");
        auto pose = mydetect->ur_move_pose;
        auto pose_str = pose2qstring(pose);
        auto cam2obj = mydetect->pub_cam2obj;
        // 添加坐标轴
        // vtkSmartPointer<vtkMatrix4x4> vmat=vtkSmartPointer<vtkMatrix4x4>::New();
        g_vtk_obj_pose = eigenMat2vtkMat(cam2obj);
        // eigenMat2vtkMat(cam2obj,vmat);
        g_obj_frame_actor->SetUserMatrix(g_vtk_obj_pose);
        g_renderer->AddActor(g_obj_frame_actor);
        ui->textEdit->append(pose_str);
        mydetect->clear_all();
    }
    catch (exception e)
    {
        ui->textEdit->append("位姿获取失败");
    }
    // 处理点云
    if (my_index < 10)
    {
        my_index++;
    }
    else
    {
        timer->stop();
        ui->textEdit->append("关闭定时器");
    }
    this->update_show();
}
// 手动模式 通过按钮更新
void MainWindow::on_pushButton_manual_clicked()
{
    if (!manual_flag)
    {
        manual_flag = true;
        if (timer->isActive())
            timer->stop();
        ui->textEdit->append("开启手动确定模式");
        this->update_show();
    }
    else
    {
        ui->textEdit->append("当前已经处于手动模式");
    }
}
// 自动模式 通过定时器更新
void MainWindow::on_pushButton_auto_clicked()
{
    if (manual_flag) // 是手动模式才能切换
    {
        manual_flag = false;
        if (!timer->isActive())
        {
            timer->start(2000); // ms
        }
        ui->textEdit->append("开启自动采集模式");
    }
    else
    {
        ui->textEdit->append("当前已经处于自动采集模式");
    }
}
// 基本流程没问题,需要处理没有点云的情况！！!! 2022-10-09晚
// 直接用 try catch 暂不处理 10-11
// 问题2 斜着拍点云时,会把内面拍进去 需要根据平面距离过滤
// 在手动模式有效! 跳过此次姿态(姿态不准时)
void MainWindow::on_pushButton_next_clicked()
{
    if (manual_flag)
    {
        try
        {
            g_renderer->RemoveActor(g_pcd_actor);
#if is_device
            // 手动复位,此处不需要
            //          vector<double> init_pose={0.421,-0.0105,0.73470,0.687,0.394,-4.525};
            //          rtde_control.moveL(init_pose);
            camera3d_capture(); // 捕获点云
#endif
            g_o3d_pcd = g_pcd_ptr;
            g_pcd_actor = o3d_pcd_to_vtk_actor(*g_o3d_pcd);
            g_pcd_actor->GetProperty()->SetColor(0.3, 0.5, 0.6);
            g_renderer->AddActor(g_pcd_actor);
            // g_o3d_pcd=make_shared<geometry::PointCloud>(pcd_ptr);
            // process pcd
            MyDetect *mydetect = new MyDetect;
            mydetect->pross_pcd(true, g_o3d_pcd, "");
            vtk_ur_pose_vec = mydetect->ur_move_pose;
            auto pose_str = pose2qstring(vtk_ur_pose_vec);
            auto cam2obj = mydetect->pub_cam2obj;
            // 添加坐标轴
            // vtkSmartPointer<vtkMatrix4x4> vmat=vtkSmartPointer<vtkMatrix4x4>::New();
            g_vtk_obj_pose = eigenMat2vtkMat(cam2obj);
            // eigenMat2vtkMat(cam2obj,vmat);
            g_obj_frame_actor->SetUserMatrix(g_vtk_obj_pose);
            g_renderer->AddActor(g_obj_frame_actor);
            ui->textEdit->append(pose_str);
            mydetect->clear_all();
            ui->textEdit->append("执行下一次采集和处理");
        }
        catch (exception e)
        {
            ui->textEdit->append("位姿获取失败");
        }
    }
    else
    {
        ui->textEdit->append("自动模式暂未添加跳过程序");
    }
    this->update_show();
}
// 2022-10-09 20:59 test OK!!
//  在手动模式有效! 机械臂执行动作
void MainWindow::on_pushButton_action_clicked()
{
    if (manual_flag)
    {
        try
        {
            // xyz 已经除于1000
#if is_device
            rtde_control.moveL(vtk_ur_pose_vec[0]);
            rtde_control.moveL(vtk_ur_pose_vec[1]);
#endif
            ui->textEdit->append("控制机械臂对接");
        }
        catch (exception e)
        {
            ui->textEdit->append("控制机械臂对接失败");
        }
    }
    else
    {
        ui->textEdit->append("自动模式暂未添加对接程序");
    }
}
// 显示坐标系与否
void MainWindow::on_actionshow_coordinate_triggered()
{
    frame_flag = !frame_flag;
    g_obj_frame_actor->SetUserMatrix(g_vtk_obj_pose);
    if (frame_flag) // true
    {
        // vtkNew<vtkRenderer> renderer;
        g_renderer->AddActor(g_obj_frame_actor);
        ui->actionshow_coordinate->setText("关闭坐标显示");
        ui->textEdit->append("开启坐标显示");
    }
    else
    {
        g_renderer->RemoveActor(g_obj_frame_actor);
        ui->actionshow_coordinate->setText("开启坐标显示");
        ui->textEdit->append("关闭坐标显示");
    }
    this->update_show();
}
void MainWindow::on_actionshow_ur5_triggered()
{
    ur5_flag = !ur5_flag;
    if (ur5_flag)
    {
        g_renderer->AddActor(g_ur5_actor);
        ui->actionshow_ur5->setText("关闭UR显示");
        ui->textEdit->append("开启UR显示");
    }
    else
    {
        g_renderer->RemoveActor(g_ur5_actor);
        ui->actionshow_ur5->setText("开启UR显示");
        ui->textEdit->append("关闭UR显示");
    }
    this->update_show();
}
void MainWindow::on_actionshow_camera_triggered()
{
    cam_flag = !cam_flag;
    if (cam_flag)
    {
        g_renderer->AddActor(g_cam_actor);
        ui->actionshow_camera->setText("关闭相机显示");
        ui->textEdit->append("开启相机显示");
    }
    else
    {
        g_renderer->RemoveActor(g_cam_actor);
        ui->actionshow_camera->setText("开启相机显示");
        ui->textEdit->append("关闭相机显示");
    }
    this->update_show();
}
// 将open3d 点云转换为 vtk
vtkNew<vtkActor> o3d_pcd_to_vtk_actor(geometry::PointCloud pcd)
{
    vtkNew<vtkPoints> points;
    vtkNew<vtkCellArray> vertices;
    vtkNew<vtkPolyData> polydata;
    for (int i = 0; i < pcd.points_.size(); i++)
    {
        auto p = pcd.points_[i];
        int x = p.x();
        int y = p.y();
        int z = p.z();
        points->InsertPoint(static_cast<vtkIdType>(i), x, y, z);
        vertices->InsertNextCell(1);
        vertices->InsertCellPoint(i);
    }
    polydata->SetPoints(points);
    polydata->SetVerts(vertices);
    vtkNew<vtkNamedColors> colors;
    // Visualize OK
    vtkNew<vtkPolyDataMapper> pointMapper;
    pointMapper->SetInputData(polydata);
    // plyMapper->SetInputConnection(points->Get());
    vtkNew<vtkActor> pointActor;
    pointActor->SetMapper(pointMapper);
    // SetColor(1,0,0)
    pointActor->GetProperty()->SetColor(colors->GetColor4d("Wheat").GetData());
    pointActor->GetProperty()->SetPointSize(2);
    return pointActor;
}
// 二维vector 转 qstring
// 输出一个预对接姿态和对接完成姿态
QString pose2qstring(vector<vector<double>> poses)
{
    QString qstr = "对接姿态:\n";
    for (auto &p : poses)
    {
        qstr += "(";
        for (auto &i : p)
        {
            qstr += QString::number(i, 'f', 5) + " ";
        }
        qstr += ")\n";
    }
// 输出机械臂姿态
#if is_device
    auto ur_pose = rtde_receive.getActualTCPPose();
    qstr += "ur 当前姿态\n";
    qstr += "(";
    for (auto &up : ur_pose)
    {
        qstr += QString::number(up, 'f', 5) + " ";
    }
    qstr += ")\n";
#endif
    return qstr;
}
// eigen Matrix4d to vtkMatrix4x4
vtkSmartPointer<vtkMatrix4x4> eigenMat2vtkMat(Eigen::Matrix4d emat)
{
    vtkSmartPointer<vtkMatrix4x4> vmat(vtkSmartPointer<vtkMatrix4x4>::New());
    for (int i = 0; i < emat.rows(); ++i)
    {
        for (int j = 0; j < emat.cols(); ++j)
        {
            double value = emat(i, j);
            vmat->SetElement(i, j, value);
        }
    }
    return vmat;
}
void MainWindow::on_action_aboutapp_triggered()
{
    QString str_app = "本应用开发于:2022-07-10,\n于2022-10-10完成主要功能。"
                      "此应用为毕业设计衍生产品,源于《基于三维点云的充电口识别和位姿获取算法研究》课题研究,\n"
                      "主要用于显示三维点云以及目标姿态,\n实现了动态更新,非阻塞显示,人机交互";
    ui->textEdit->append(str_app);
}
void MainWindow::on_action_aboutme_triggered()
{
    QString str_me = "作者:熊志豪\n"
                     "邮箱:dorob@qq.com";
    ui->textEdit->append(str_me);
}
// 连接机械臂
void MainWindow::on_connect_camera3d_triggered()
{
    ui->textEdit->append("已默认连接相机");
}
// 机械臂复位
void MainWindow::on_reset_ur5_triggered()
{
    try
    {
        vector<double> init_pose = {0.436, -0.130, 0.630, 2.074, -0.052, 2.256};
#if is_device
        // rtde_control.moveL(vtk_ur_pose_vec[0]);
        rtde_control.moveL(init_pose); // vtk_ur_pose_vec
#endif
        ui->textEdit->append("机械臂复位");
    }
    catch (exception e)
    {
        ui->textEdit->append("机械臂复位失败");
    }
}
// 清空文本显示
void MainWindow::on_actionclear_text_triggered()
{
    ui->textEdit->clear();
}
void MainWindow::on_actionoffset_triggered()
{
    if (ui->actionoffset->text() == "打开手眼补偿")
    {
        ui->actionoffset->setText("关闭手眼补偿");
        ui->textEdit->append("开启手眼补偿");
        ui->xSlider->setEnabled(true);
        ui->ySlider->setEnabled(true);
        ui->zSlider->setEnabled(true);
        ui->rxSlider->setEnabled(true);
        ui->rySlider->setEnabled(true);
        ui->rzSlider->setEnabled(true);
    }
    else
    {
        ui->actionoffset->setText("打开手眼补偿");
        ui->textEdit->append("关闭手眼补偿");
        ui->xSlider->setEnabled(false);
        ui->ySlider->setEnabled(false);
        ui->zSlider->setEnabled(false);
        ui->rxSlider->setEnabled(false);
        ui->rySlider->setEnabled(false);
        ui->rzSlider->setEnabled(false);
    }
}
/*————————————————————————————————————————————————————————————
// 这部分调节需要关联 myDetect.cpp 中的矩阵变换, 待添加
或者 直接对传出的位姿进行补偿
补偿矩阵计算,重新计算末端姿态
补偿矩阵 * 机械臂基坐标系* end2obj = obj
—————————————————————————————————————————————————————————————*/
void MainWindow::on_xSlider_valueChanged(int value)
{
    double x = ui->xSlider->value() - 50;
    double cur_x = g_src_x + x / 5.0;
    double cur_y = g_ur5_actor->GetPosition()[1];
    double cur_z = g_ur5_actor->GetPosition()[2];
    // ui->textEdit->append("x补偿: "+QString::number(x));
    g_ur5_actor->SetPosition(cur_x, cur_y, cur_z);
    // g_ur5_actor->AddPosition(x/10.0,0,0);// 相对移动
    this->update_show();
}
void MainWindow::on_ySlider_valueChanged(int value)
{
    auto y = ui->ySlider->value() - 50;
    double cur_y = g_src_y + y / 5.0;
    double cur_x = g_ur5_actor->GetPosition()[0];
    double cur_z = g_ur5_actor->GetPosition()[2];
    // ui->textEdit->append("y补偿: "+QString::number(y));
    g_ur5_actor->SetPosition(cur_x, cur_y, cur_z);
    this->update_show();
}
void MainWindow::on_zSlider_valueChanged(int value)
{
    auto z = ui->zSlider->value() - 50;
    double cur_z = g_src_z + z / 5.0;
    double cur_x = g_ur5_actor->GetPosition()[0];
    double cur_y = g_ur5_actor->GetPosition()[1];
    // ui->textEdit->append("z补偿: "+QString::number(z));
    g_ur5_actor->SetPosition(cur_x, cur_y, cur_z);
    this->update_show();
}
void MainWindow::on_rxSlider_valueChanged(int value)
{
    double rx = ui->rxSlider->value() - 50;
    rx /= 100.0;
    // ui->textEdit->append("rx补偿: "+QString::number(rx));
    g_ur5_actor->RotateX(rx);
    this->update_show();
}
void MainWindow::on_rySlider_valueChanged(int value)
{
    double ry = ui->rySlider->value() - 50;
    ry /= 100.0;
    // ui->textEdit->append("ry补偿: "+QString::number(ry));
    g_ur5_actor->RotateY(ry);
    this->update_show();
}
void MainWindow::on_rzSlider_valueChanged(int value)
{
    double rz = ui->rzSlider->value() - 50;
    rz /= 100.0;
    // ui->textEdit->append("rz补偿: "+QString::number(rz));
    g_ur5_actor->RotateZ(rz);
    this->update_show();
}
QString writeVtkMatrix4x4ToFile(const vtkMatrix4x4 *m, string filename)
{
    ofstream fout(filename);
    QString mat_str = "";
    if (!fout)
    {
        cout << "File Not Opened" << endl;
        return "";
    }
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            double value = m->GetElement(i, j);
            fout << value << "  ";
            mat_str += QString::number(value) + " ";
        }
        mat_str += "\n";
        fout << endl;
    }
    fout << endl;
    fout.close();
    return mat_str;
}
void MainWindow::on_actionsave_offset_triggered()
{
    auto vtk_mat = g_ur5_actor->GetMatrix();
    auto mat_str = writeVtkMatrix4x4ToFile(vtk_mat, "./offset.txt");
    ui->textEdit->append("保存手眼矩阵到 offset.txt\n"
                         "————————————————");
    ui->textEdit->append(mat_str);
}
// 切换模式 充电或者钻孔
void MainWindow::on_actionswitch_mode_triggered()
{
    if (ui->pushButton_action->text() == "充电")
    {
        ui->textEdit->append("切换到钻孔模式");
        ui->pushButton_action->setText("钻孔");
        MainWindow::setWindowTitle("钻孔识别");
    }
    else
    {
        ui->textEdit->append("切换到充电模式");
        ui->pushButton_action->setText("充电");
        MainWindow::setWindowTitle("充电口识别");
    }
}
