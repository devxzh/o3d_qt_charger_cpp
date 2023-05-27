#ifndef MAINWINDOW_H
#define MAINWINDOW_H
// UR 头文件要在前,否则报winsock错误
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <QMainWindow>
#include <QTimer>
#include "my_vtk.h"
#include "my_detect.hpp"
#include "scanner_api.h"
//#include "o3d.hpp"
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE
// 将open3d 点云转换为 vtk
vtkNew<vtkActor> o3d_pcd_to_vtk_actor(geometry::PointCloud pcd);
// 二维vector 转qstring
QString pose2qstring(vector<vector<double>> poses);
vtkSmartPointer<vtkMatrix4x4> eigenMat2vtkMat(Eigen::Matrix4d emat);
class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void update_show(string append_str="");
    void timer_update(void);
private slots:
    void on_pushButton_manual_clicked();
    void on_actionshow_coordinate_triggered();
    void on_actionshow_ur5_triggered();
    void on_actionshow_camera_triggered();
    void on_pushButton_auto_clicked();
    void on_pushButton_next_clicked();
    void on_pushButton_action_clicked();
    void on_action_aboutapp_triggered();
    void on_action_aboutme_triggered();
    void on_connect_camera3d_triggered();
    void on_reset_ur5_triggered();
    void on_actionclear_text_triggered();
    void on_actionoffset_triggered();
    void on_xSlider_valueChanged(int value);
    void on_ySlider_valueChanged(int value);
    void on_zSlider_valueChanged(int value);
    void on_rxSlider_valueChanged(int value);
    void on_rySlider_valueChanged(int value);
    void on_rzSlider_valueChanged(int value);
    void on_actionsave_offset_triggered();
    void on_actionswitch_mode_triggered();
private:
    Ui::MainWindow *ui;
    QTimer *timer;
    // global varible 全局用vtkSmartPointer,局部用vtkNew
    vtkSmartPointer<vtkRenderer> g_renderer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> g_renderWindow;
    vtkSmartPointer<vtkPolyData> g_pcd;
    vtkSmartPointer<vtkActor> g_pcd_actor,g_ur5_actor,g_cam_actor,g_obj_frame_actor;
    vtkSmartPointer<vtkMatrix4x4> g_vtk_obj_pose;
    PcdPtr g_o3d_pcd;
    Eigen::Matrix4d g_obj_pose;
    int my_index;       // 用于读取文件记录
    string text_info;  // 显示调试信息
    vector<vector<double> > vtk_ur_pose_vec;
    bool ur5_flag;
    bool frame_flag;
    bool cam_flag;
    bool manual_flag;
};
#endif // MAINWINDOW_H
