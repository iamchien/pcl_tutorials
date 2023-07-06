#include <bits/stdc++.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>

#include <vtkPolyDataReader.h> // for vtkPolyDataReader
#include <vtkPLYWriter.h> // for vtkPLYWriter

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.vtk output.ply\n", argv[0]);
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Convert a VTK file to PLY format. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .vtk and .ply files
  std::vector<int> vtk_file_indices = parse_file_extension_argument (argc, argv, ".vtk");
  std::vector<int> ply_file_indices = parse_file_extension_argument (argc, argv, ".ply");
  if (vtk_file_indices.size () != 1 || ply_file_indices.size () != 1)
  {
    print_error ("Need one input VTK file and one output PLY file.\n");
    return (-1);
  }

  // Load the input file
  vtkSmartPointer<vtkPolyData> polydata;
  vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New ();
  reader->SetFileName (argv[vtk_file_indices[0]]);
  reader->Update ();
  polydata = reader->GetOutput ();
  print_info ("Loaded %s with %lu points/vertices.\n", argv[vtk_file_indices[0]], polydata->GetNumberOfPoints ());

  // Convert to PLY and save
  vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New ();
  writer->SetInputData (polydata);
  writer->SetArrayName ("Colors");
  writer->SetFileTypeToASCII ();
  writer->SetFileName (argv[ply_file_indices[0]]);
  writer->Write ();

  return (0);
}

