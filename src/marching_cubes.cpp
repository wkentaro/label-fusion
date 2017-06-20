#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/marching_cubes_hoppe.h>

int
main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
  {
    std::cout << "ERROR: couldn't find file" << std::endl;
    return 1;
  }
  else
  {
    std::cout << "loaded" << std::endl;

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
    tree1->setInputCloud(cloud);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree1);
    ne.setKSearch(10);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(cloud_with_normals);

    std::cout << "begin marching cubes reconstruction" << std::endl;

    pcl::MarchingCubesHoppe<pcl::PointNormal> mc;
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
    mc.setIsoLevel(0.0f);
    mc.setGridResolution(50, 50, 50);
    mc.setPercentageExtendGrid(0.0f);
    mc.setInputCloud(cloud_with_normals);
    mc.setSearchMethod(tree);
    mc.reconstruct(*triangles);

    std::cout << triangles->polygons.size() << " triangles created" << std::endl;

    pcl::PolygonMesh output;
    mc.reconstruct(output);
    std::string out_file("marching_cubes.ply");
    pcl::io::savePLYFile(out_file, output);
  }
}
