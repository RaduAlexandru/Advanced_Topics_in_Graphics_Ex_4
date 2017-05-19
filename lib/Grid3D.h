#ifndef ATCG1_GRID3D_H
#define ATCG1_GRID3D_H

#include <Eigen/Core>

class Grid3D {
private:
    Eigen::MatrixXd normals_field;

    Eigen::VectorXi map_points_to_lin_idx;

    std::vector<Eigen::VectorXi> points_indices;
    std::vector<Eigen::VectorXi> normals_indices;

    Eigen::Vector3i lin_idx_to_coord(const int lin_idx, const int gridn);

    int coord_to_lin_idx(const Eigen::Vector3i &coord, const int gridn);

    Eigen::Vector3d coord_to_voxel_center(const Eigen::Vector3i &coord, const int gridn, const double abs_voxelsize);

    Eigen::Vector3i point_to_coord(const Eigen::Vector3d &point, const int gridn, const double abs_voxelsize);

    void construct_band();

    void construct_full_corners_and_edges();

public:
    Eigen::MatrixXd points;
    Eigen::MatrixXd normals;

    int gridn;
    int num_voxel;
    double abs_voxelsize;
    double unit_scale;

    Eigen::RowVector3d points_mean;

    Eigen::MatrixXd grid_points;
    Eigen::MatrixXi grid_edges;

    Eigen::MatrixXi band_edges;

    Grid3D(const Eigen::MatrixXd &points, const int gridn, double scale);

    void visualize_edges(igl::viewer::Viewer &viewer, const Eigen::RowVector3d color = Eigen::RowVector3d(0.5, 0.5, 0.5));

    Eigen::MatrixXd volume_to_slice(const int slice_index, const int dim, const Eigen::VectorXd &data);

    void slice_to_volume(const int slice_index, const int dim, const Eigen::MatrixXd &slice, Eigen::MatrixXd &data);

    void smooth_normals(Eigen::MatrixXd &normals, const double sigma);

    Eigen::Matrix<short, Eigen::Dynamic, 1> normal_divergence();
};


#endif //ATCG1_GRID3D_H
