#include <bits/stdc++.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.vtk output.obj\n", argv[0]);
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Convert a VTK file to OBJ format. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .vtk and .obj files
  std::vector<int> vtk_file_indices = parse_file_extension_argument (argc, argv, ".vtk");
  std::vector<int> obj_file_indices = parse_file_extension_argument (argc, argv, ".obj");
  if (vtk_file_indices.size () != 1 || obj_file_indices.size () != 1)
  {
    print_error ("Need one input VTK file and one output OBJ file.\n");
    return (-1);
  }

  // Load the input file
  PolygonMesh mesh;
  loadPolygonFileVTK (argv[vtk_file_indices[0]], mesh);

  saveOBJFile (argv[obj_file_indices[0]], mesh);
  return (0);
}

