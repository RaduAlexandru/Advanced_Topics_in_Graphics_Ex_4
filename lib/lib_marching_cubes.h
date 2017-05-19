#ifndef ATCG1_LIB_MARCHING_CUBES_H
#define ATCG1_LIB_MARCHING_CUBES_H
#include <Eigen/Core>

struct Triangle{
    Eigen::Vector3d vertices[3];
};

/*Eigen::Vector3i lin_idx_to_coord(const int n, const int gridn);

int coord_to_lin_idx(const Eigen::Vector3i &coord, const int gridn);

Eigen::Vector3d coord_to_voxel_center(const Eigen::Vector3i &coord, const int gridn, const double abs_voxelsize);

Eigen::Vector3i point_to_coord(const Eigen::Vector3d &point, const int gridn, const double abs_voxelsize);*/

void construct_sdf_field(const int gridn, const double abs_voxelsize, Eigen::VectorXd &sdf_field);

Eigen::Vector3d interpolate(const double isovalue, const Eigen::Vector3d &p1, const double sdf1, const Eigen::Vector3d &p2, double sdf2);

std::vector<Triangle> marching_cubes(const Eigen::VectorXd &sdf_field, const int gridn, const double abs_voxelsize, const double isovalue);

#endif //ATCG1_LIB_MARCHING_CUBES_H
