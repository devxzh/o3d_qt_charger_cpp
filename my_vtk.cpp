#include "my_vtk.h"

// 返回一个actor ,显示多个图形
vtkNew<vtkActor> load_mesh(std::string ply_name)
{
    vtkNew<vtkNamedColors> colors;
    vtkNew<vtkPLYReader> reader;
    reader->SetFileName(ply_name.c_str());

    // Visualize
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(reader->GetOutputPort());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(colors->GetColor3d("DarkGray").GetData());

    return actor;
}
