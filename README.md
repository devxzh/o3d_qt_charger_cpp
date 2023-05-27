# o3d_qt_charger_cpp
《基于三维点云的充电口识别及其位姿获取算法研究》论文的代码
### 开发环境说明
| 作用 |软件 | 版本 |
|:--:|:--:|:--:|
| 系统 | Windows 11 | 21H2 |
| 编程语言 | C++ | C++ 14 |
| 图形界面 | QT | 6.2.3 |
| 点云处理 | Open3D | 0.15.2 |
| 点云可视化 | VTK | 9.1 |
| 相机接口 | Comatrix | 1.9.4 |
| 优傲机器人 | ur-rtde | 1.5.1 |
| 库编译工具 | MSVC | 16.7 |
### 注意事项
1. QT项目不要使用中文路径
2. 使用QT Creater 打开CmakeLists.txt，即可打开工程，对应的库需要更改为自己配置的路径和版本；
3. 依赖 comatrix api，VTK，Open3D，ur-rtde，其版本见 软件环境说明.jpg；
4. 该项目也可在Ubuntu下运行，需自行安装依赖和库。
### 机器人结构图
![机器人结构图](https://github.com/devxzh/o3d_qt_charger_cpp/blob/main/imgs/robots.png)
### 上位机界面
![上位机界面](https://github.com/devxzh/o3d_qt_charger_cpp/blob/main/imgs/app_gui.png)