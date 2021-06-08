#include <iostream>
#include <ostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointCloud<pcl::PointXYZ> PCXYZ;
typedef pcl::PointCloud<pcl::Normal> PCNormal;
typedef pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> NormalEstimation;
typedef pcl::search::KdTree<pcl::PointXYZ> KdTree;

int main(int argc, char** argv)
{
  std::string plyFilename = "/home/jaired/Datasets/ABQ-215/tests/features/siftgpu5k/sparse3D.ply";
  double searchRadius = 100.0;
  if (argc == 2)
    plyFilename = std::string(argv[1]);
  else if (argc == 3)
    searchRadius = std::stod(argv[2]);

  PCXYZ::Ptr cloud(new PCXYZ);

  int rc = pcl::io::loadPLYFile(plyFilename, *cloud);
  if (rc)
    std::cout << "Can't read PLY file." << std::endl;

  NormalEstimation ne;
  ne.setInputCloud(cloud);

  KdTree::Ptr tree (new KdTree());
  ne.setSearchMethod(tree);

  // Output datasets
  PCNormal::Ptr cloud_normals (new PCNormal);

  // Use all neighbors in a sphere of radius = searchRadius
  ne.setRadiusSearch(searchRadius);

  // Compute the features
  ne.compute(*cloud_normals);

  // Counts should match
  std::cout << "Normal count: " << cloud_normals->size() << std::endl;
  std::cout << "Point count: " << cloud->size() << std::endl;

  std::ofstream outstream("normals.xyz");
  for (pcl::Normal& norm : *cloud_normals)
    outstream << norm._Normal::normal_x << " " << norm._Normal::normal_y << " "
              << norm._Normal::normal_z << std::endl;
  outstream.close();

  return 0;
}
