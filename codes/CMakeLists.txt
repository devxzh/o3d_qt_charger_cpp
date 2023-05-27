cmake_minimum_required(VERSION 3.5)
project(qt_o3d_gui VERSION 0.1 LANGUAGES CXX)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(VTK_DIR D:/PCL_112/3rdParty/VTK)
find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)
find_package(VTK REQUIRED)
find_package(Open3D REQUIRED)
find_package(ur_rtde REQUIRED)
include_directories(
    D:/ur_rtde_151/include
    D:/ur_rtde_151/include/ur_rtde
    D:/PCL_112/3rdParty/VTK/include
    D:/PCL_112/3rdParty/VTK/include/vtk-9.1
    D:/open3d_0152/include
    D:/open3d_0152/include/open3d
    )
set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        #o3d.hpp
        mainwindow.h
        mainwindow.ui
        my_vtk.h
        my_vtk.cpp
        myDetect.hpp
        myDetect.cpp
        scanner_api.h
)
if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(qt_o3d_gui
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET qt_o3d_gui APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(qt_o3d_gui SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(qt_o3d_gui
            ${PROJECT_SOURCES}
        )
    endif()
endif()
target_link_libraries(qt_o3d_gui
    PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(qt_o3d_gui
    PRIVATE ${VTK_LIBRARIES})
target_link_libraries(qt_o3d_gui PRIVATE Open3D::Open3D)
target_link_libraries(qt_o3d_gui PRIVATE ur_rtde::rtde)
target_link_libraries(qt_o3d_gui PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libscan_release.lib)
set_target_properties(qt_o3d_gui PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)
if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(qt_o3d_gui)
endif()
