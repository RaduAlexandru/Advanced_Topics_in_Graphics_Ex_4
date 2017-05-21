#include <cstdlib>
#include <iostream>
#include <math.h>

//set this to supress libigl viewer help
#define IGL_VIEWER_VIEWER_QUIET

#include <igl/viewer/Viewer.h>
#include <igl/readPLY.h>
#include <Eigen/Core>
#include <TouchExpand.h>
#include <iterator>

#include "Grid3D.h"
#include "lib_marching_cubes.h"
/**
 * \brief Scales the data X into the unit hypercube
 *
 * sometimes data has a funny scale and it is outside of the libigl-viewer-frustrum
 * this is a convenience function to scale the data into the unit hypercube
 *
 * \params[in/out] X The data points organized as a n x d matrix
 */
void scale_to_unit_cube(Eigen::MatrixXd &X, double &s)
{
    if (s == 0)
    {
        s = 1.0 / X.colwise().maxCoeff().maxCoeff();
    }

    X = (X.array() * s).matrix();
}

// generate random number for the range (min, max)
double irand(int min, int max)
{
    return ((double)rand() / ((double)RAND_MAX + 1.0)) * (max - min) + min;
}

/**
* \brief Uniformly sample the parameterspace of cylinder and map to sphere
*
* what it does:
*   -uniformly sample the parameter space of a cylinder with h, phi, r=1;
*   -map these parameters to the parameter space of the sphere
*   -construct the associated points on the sphere
*
* TODO:
*   -generate samples
*   -map them to the other space
*   -generate points
*
* \param[in] num_samples The number of samples to generate
*/
Eigen::MatrixXd create_sphere(const int num_samples)
{
    Eigen::MatrixXd out_points = Eigen::MatrixXd::Zero(num_samples, 3);
    int samples_per_parameter = (int) std::sqrt(num_samples);


    for (size_t i = 0; i < num_samples; i++) {
      float theta = 2*M_PI*irand(0,1);
      out_points(i,2) = irand(-1,1);
      out_points(i,0) = sqrt(1-out_points(i,2)*out_points(i,2))*cos(theta);
      out_points(i,1) = sqrt(1-out_points(i,2)*out_points(i,2))*sin(theta);
    }

    return out_points;
}

void show_coordinate_system(igl::viewer::Viewer &viewer)
{
    Eigen::Matrix3d origin = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d axis = Eigen::Matrix3d::Identity();
    viewer.data.add_edges(origin, axis, axis);
}

/**
 * \brief First exercise
 *
 * what it does:
 *  -uniformly sample the parameter domain of a cylinder and map these to the sphere
 *  -construct a 3D grid which contains the shpere
 *  -smooth the normals of the sphere
 *  -compute the divergence for each voxel
 *  -call Touch Expand to fill holes
 *  -call marching cubes to reconstruct the surface
 *  -V is (n x 3)
 *
 * TODO:
 *  -implement this exercise
 *  -complete the function create_sphere
 *  -complete the function smooth_normals
 *  -complete the function normal_divergence
 *
 */
void exercise1(){
    const int gridn = 100;

    Eigen::MatrixXd points = create_sphere(8000);
    Eigen::MatrixXd normals = points.rowwise().normalized();

    Grid3D grid = Grid3D(points, gridn, 0.6);

    grid.smooth_normals(normals, 0.009);
    Eigen::Matrix<short, Eigen::Dynamic, 1> tLinks = grid.normal_divergence();

    int dimensions[3] = {grid.gridn, grid.gridn, grid.gridn};

    TouchExpand *te = new TouchExpand(dimensions, NEIGH26, 1, tLinks.data(), NULL);
    te->InitializeCoarseToFine();
    te->MaxFlow(1);

    unsigned char *result = new unsigned char[grid.num_voxel];
    te->GetResult(result);

    Eigen::VectorXd sdf_field(grid.num_voxel);
    for(int i=0; i< grid.num_voxel; i++)
    {
        sdf_field(i) = result[i];
    }

    std::vector<Triangle> triangle_soup = marching_cubes(sdf_field, grid.gridn, grid.abs_voxelsize, 0);
    Eigen::MatrixXd V_rc(triangle_soup.size() * 3, 3);
    Eigen::MatrixXi F_rc(triangle_soup.size(), 3);

    for(int i = 0; i < triangle_soup.size(); i++)
    {
        for(int j = 0; j < 3; j++)
        {
            V_rc.row(3 * i + j) = triangle_soup[i].vertices[j];// / grid.unit_scale;
            F_rc(i, j) = 3 * i + j;
        }
    }

    igl::viewer::Viewer viewer;
    viewer.data.set_mesh(V_rc, F_rc);
    viewer.data.set_points(grid.points, Eigen::RowVector3d(0.0, 1.0, 0.0));
    // grid.visualize_edges(viewer);
    // viewer.data.add_edges(points, points + normals * 0.1, Eigen::RowVector3d(1.0, 0.0, 0.0));
    viewer.core.point_size = 5;
    show_coordinate_system(viewer);
    viewer.launch();
}

/**
 * \brief Second exercise
 *
 * what it does:
 *  -load the santa clause point cloud
 *  -construct a 3D grid which contains the point cloud
 *  -smooth the normals of the point cloud
 *  -compute the divergence for each voxel
 *  -call Touch Expand to fill holes
 *  -call marching cubes to reconstruct the surface
 *  -V is (n x 3)
 *
 */
void exercise2(const std::string &filename){
    Eigen::MatrixXd V;
    Eigen::MatrixXi F; // faces is only a dummy for loading
    Eigen::MatrixXd NORMALS;
    Eigen::MatrixXd UV;// UV is empty

    bool load_success_source = igl::readPLY(filename, V, F, NORMALS, UV);

    if (!load_success_source)
    {
        std::cerr << "could not load file: " << filename << std::endl;
        return;
    }

    double s = 0.0;
    scale_to_unit_cube(V, s);
    const int gridn = 100;

    Eigen::MatrixXd points = V;
    Eigen::MatrixXd normals = NORMALS;

    Grid3D grid = Grid3D(points, gridn, 0.8);

    grid.smooth_normals(normals, 0.001);
    Eigen::Matrix<short, Eigen::Dynamic, 1> tLinks = grid.normal_divergence();

    int dimensions[3] = {grid.gridn, grid.gridn, grid.gridn};

    TouchExpand *te = new TouchExpand(dimensions, NEIGH26, 1, tLinks.data(), NULL);
    te->InitializeCoarseToFine();
    te->MaxFlow(1);

    unsigned char *result = new unsigned char[grid.num_voxel];
    te->GetResult(result);

    Eigen::VectorXd sdf_field(grid.num_voxel);
    for(int i=0; i< grid.num_voxel; i++)
    {
        sdf_field(i) = result[i];
    }

    std::vector<Triangle> triangle_soup = marching_cubes(sdf_field, grid.gridn, grid.abs_voxelsize, 0);
    Eigen::MatrixXd V_rc(triangle_soup.size() * 3, 3);
    Eigen::MatrixXi F_rc(triangle_soup.size(), 3);

    for(int i = 0; i < triangle_soup.size(); i++)
    {
        for(int j = 0; j < 3; j++)
        {
            V_rc.row(3 * i + j) = triangle_soup[i].vertices[j];
            F_rc(i, j) = 3 * i + j;
        }
    }

    igl::viewer::Viewer viewer;
    viewer.data.set_mesh(V_rc, F_rc);
    viewer.data.set_points(grid.points, Eigen::RowVector3d(0.0, 1.0, 0.0));
    //grid.visualize_edges(viewer);
    //viewer.data.add_edges(grid.points, grid.points + normals * 0.1, Eigen::RowVector3d(1.0, 0.0, 0.0));
    viewer.core.point_size = 2;
    show_coordinate_system(viewer);
    viewer.launch();
}


/**
* \brief The main function called when running this program
*
* what it does:
*  -check provided filenames
*  -run both exercises in a row
*
*  \param[in] argc The number of arguments to the binary
*  \param[in] argv The array of arguments to the binary
*/
int main(int argc, char *argv[])
{
    std::string filename;

    if (argc == 2)
    {
        filename = argv[1];
    }
    else
    {
        std::cerr << "please call assignmentsheet4 like this: " << std::endl;
        std::cerr << "./bin/assignmentsheet4 data/santa_normals.ply" << std::endl;
        return EXIT_FAILURE;
    }

    exercise1();
    // exercise2(filename);
	return EXIT_SUCCESS;
}
