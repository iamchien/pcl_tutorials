#include <bits/stdc++.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>

#include <vtkPLYReader.h> // for vtkPLYReader
#include <vtkPolyDataWriter.h> // for vtkPolyDataWriter

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.ply output.vtk\n", argv[0]);
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Convert a PLY file to VTK format. For more information, use: %s -h\n", argv[0]);

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
    print_error ("Need one input PLY file and one output VTK file.\n");
    return (-1);
  }

  // Load the input file
  vtkSmartPointer<vtkPolyData> polydata;
  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New ();
  reader->SetFileName (argv[ply_file_indices[0]]);
  reader->Update ();
  polydata = reader->GetOutput ();
  print_info ("Loaded %s with %lu points/vertices.\n", argv[ply_file_indices[0]], polydata->GetNumberOfPoints ());

  // Convert to VTK and save
  vtkSmartPointer<vtkPolyDataWriter> writer = vtkSmartPointer<vtkPolyDataWriter>::New ();
  writer->SetInputData (polydata);
  writer->SetFileName (argv[vtk_file_indices[0]]);
  writer->SetFileTypeToBinary ();
  writer->Write ();

  return (0);
}

