#ifndef ATCG1_LIB_CELL_STRUCTURE_H
#define ATCG1_LIB_CELL_STRUCTURE_H
#include "Eigen/Core"

Eigen::Vector3i lin_idx_to_coord(const int n, const int gridn);

int coord_to_lin_idx(const Eigen::Vector3i &coord, const int gridn);

Eigen::Vector3d coord_to_voxel_center(const Eigen::Vector3i &coord, const int gridn, const double abs_voxelsize);

Eigen::Vector3i point_to_coord(const Eigen::Vector3d &point, const int gridn, const double abs_voxelsize);

void show_voxel_grid(const Eigen::MatrixXd &points, const double abs_voxelsize, Eigen::MatrixXd &grid_points, Eigen::MatrixXi &out_edges);

std::vector<Eigen::VectorXi> points_to_lin_idx(const Eigen::MatrixXd &points, const double abs_voxelsize, Eigen::VectorXi &indices);
#endif //ATCG1_LIB_CELL_STRUCTURE_H
