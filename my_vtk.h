#ifndef MY_VTK_H
#define MY_VTK_H
#include <iostream>
#include <string>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtkNew.h>
#include <vtkPLYReader.h>
#include <vtkPolyData.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>
#include <vtkAxesActor.h>
#include <vtkTransform.h>
// 返回一个actor ,显示多个图形
vtkNew<vtkActor> load_mesh(std::string ply_name);
#endif // MY_VTK_H
