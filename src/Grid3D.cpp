#include <igl/viewer/Viewer.h>
#include "Grid3D.h"
#include "lib_convolution.h"

void scale_to_unit_cube1(Eigen::MatrixXd &X, double &s)
{
    if (s == 0)
    {
        s = 1.0 / X.colwise().maxCoeff().maxCoeff();
    }

    X = (X.array() * s).matrix();
}

/**
* \brief Constructor for the 3D-Grid
*
* what it does:
*   -center the data to the origin
*   -scale the data into the -1,1-cube
*   -apply the userdefined scaling
*   -set and output some additional parameters
*   -construct the grid_points and grid_edges for visualization
*
* \param[in] points The point cloud: n x 3
* \param[in] gridn The number of voxels per dimension
* \param[in] scale The user-defined scaling
*/
Grid3D::Grid3D(const Eigen::MatrixXd &points, const int gridn, const double scale)
{
    points_mean = points.colwise().mean();
    this->points = (points.rowwise() - points_mean);

    unit_scale = 0.0;
    scale_to_unit_cube1(this->points, unit_scale);

    unit_scale *= scale;
    scale_to_unit_cube1(this->points, unit_scale);

    assert(std::max(this->points.maxCoeff(), -this->points.minCoeff()) <= 1.0);

    this->gridn = gridn;
    this->num_voxel = gridn * gridn * gridn;
    this->abs_voxelsize = 2.0 / (double) gridn;


    std::cout << "construct cube with sidelength: " << gridn * abs_voxelsize << std::endl;
    std::cout << "              number of voxels: " << num_voxel << std::endl;
    std::cout << "               smallest corner: " << Eigen::RowVector3d(0.0, 0.0, 0.0) << std::endl;
    std::cout << "                largest corner: " << Eigen::RowVector3d(gridn * abs_voxelsize, gridn * abs_voxelsize, gridn * abs_voxelsize) << std::endl;


    std::cout << "points are contained in bbox: " << this->points.colwise().minCoeff() << " to " << this->points.colwise().maxCoeff() << std::endl;

    construct_full_corners_and_edges();
    construct_band();
}

/**
* \brief Constructs the grid_points and grid_edges for visualization
*/
void Grid3D::construct_full_corners_and_edges()
{
    double I = abs_voxelsize;
    double O = 0.0;

    grid_points.resize(num_voxel * 8, 3);
    grid_edges.resize(num_voxel * 12, 2);

    for(int i=0; i < num_voxel; i++)
    {
        Eigen::RowVector3d center = coord_to_voxel_center(lin_idx_to_coord(i, gridn), gridn, abs_voxelsize);

        Eigen::MatrixXd voxel(8, 3);
        voxel << center + Eigen::RowVector3d(O, O, O),
                 center + Eigen::RowVector3d(I, O, O),
                 center + Eigen::RowVector3d(I, I, O),
                 center + Eigen::RowVector3d(O, I, O),
                 center + Eigen::RowVector3d(O, O, I),
                 center + Eigen::RowVector3d(I, O, I),
                 center + Eigen::RowVector3d(I, I, I),
                 center + Eigen::RowVector3d(O, I, I);

        Eigen::MatrixXi edges(12, 2);
        edges << 0, 1,
                1, 2,
                2, 3,
                3, 0,
                4, 5,
                5, 6,
                6, 7,
                7, 4,
                0, 4,
                1, 5,
                2, 6,
                3, 7;

        edges = (edges.array() + i * 8).matrix();

        grid_points.block(i * 8, 0, 8, 3) = (voxel).rowwise() - Eigen::RowVector3d(abs_voxelsize, abs_voxelsize, abs_voxelsize) / 2.0;

        grid_edges.block(i * 12, 0, 12, 2) = edges;
    }
}

/**
* \brief Constructs the voxel band around the point cloud and maps each point to the voxel it is contained in
*
* what it does:
*   -for each voxel, save the point index it contains
*   -save the edges of the voxelband around the point cloud
*/
void Grid3D::construct_band()
{
    double gridn1 = gridn + 1;

    map_points_to_lin_idx.resize(points.rows());
    band_edges.resize(points.rows() * 12, 2);

    std::vector<std::vector<int>> cell_indices(gridn1 * gridn1 * gridn1);

    for(int i=0; i < points.rows(); i++)
    {
        map_points_to_lin_idx(i) = coord_to_lin_idx(point_to_coord(points.row(i), gridn, abs_voxelsize), gridn);
        cell_indices[map_points_to_lin_idx(i)].push_back(i);
        band_edges.block(i * 12, 0, 12, 2) = grid_edges.block(map_points_to_lin_idx(i) * 12, 0, 12, 2);
    }

    for(int i=0; i < cell_indices.size(); i++)
    {
        Eigen::VectorXi local_index_set(cell_indices[i].size());
        for(int j=0; j < cell_indices[i].size(); j++) {
            local_index_set(j) = cell_indices[i][j];
        }
        points_indices.push_back(local_index_set);
    }
}

/**
* \brief Map the linear index: lin_idx to voxel coordinates (i, j, k)
*
* what it does:
*   -lin_idx = i + j* gridn + k * gridn*gridn
*   -infer i, j and k from lin_idx
*
* TODO:
*   -implement this mapping
*
* \param[in] lin_idx The linear index of a voxel
* \param[in] gridn The number of voxels in each dimension
*/
Eigen::Vector3i Grid3D::lin_idx_to_coord(const int lin_idx, const int gridn)
{
    int i = 0, j = 0, k = 0;
    // lin_idx = i + j* gridn + k * gridn*gridn

    k = lin_idx / (gridn * gridn);
    j = (lin_idx / gridn) % gridn;
    i = lin_idx % gridn;

    return Eigen::Vector3i(i, j, k);
}


/**
* \brief Map the voxel coordinates (i, j, k) to the linear index lin_idx
*
* what it does:
*   -lin_idx = i + j* gridn + k * gridn*gridn
*
* TODO:
*   -implement this mapping
*
* \param[in] coord The voxel coordinates (i, j, k) of a voxel
* \param[in] gridn The number of voxels in each dimension
*/
int Grid3D::coord_to_lin_idx(const Eigen::Vector3i &coord, const int gridn)
{
    int lin_idx = 0;
    lin_idx = coord(0) + coord(1)* gridn + coord(2) * gridn*gridn;
    return lin_idx;
}

/**
* \brief Map the voxel coordinates (i, j, k) to the voxel center
*
* what it does:
*   -compute the half_gridsize of the voxel grid
*   -shift the voxel coordinate to the positive quadrant (by half_gridsize)
*   -map the coordinate i to the x value of the center of its voxel
*   -map the coordinate j to the y value of the center of its voxel
*   -map the coordinate k to the z value of the center of its voxel
*
* TODO:
*   -implement this mapping
*
* \param[in] coord The voxel coordinates (i, j, k) of a voxel
* \param[in] gridn The number of voxels in each dimension
* \param[in] abs_voxelsize The absolute sidelength of a voxel
*/
Eigen::Vector3d Grid3D::coord_to_voxel_center(const Eigen::Vector3i &coord, const int gridn, const double abs_voxelsize)
{
    double x = 0.0, y = 0.0, z = 0.0;
    x=coord(0)*gridn+abs_voxelsize/2.0;
    y=coord(1)*gridn+abs_voxelsize/2.0;
    z=coord(2)*gridn+abs_voxelsize/2.0;

    return Eigen::Vector3d(x, y, z);
}

/**
* \brief Map a point to the voxel coordinate (i, j, k) it is contained in
*
* what it does:
*   -compute the half_gridsize of the voxel grid
*   -shift the point coordinate to the positive quadrant (by half_gridsize)
*   -map the coordinate x to the i value
*   -map the coordinate y to the y value
*   -map the coordinate z to the z value
*
* TODO:
*   -implement this mapping
*
* \param[in] point The point coordinates (x, y, z)
* \param[in] gridn The number of voxels in each dimension
* \param[in] abs_voxelsize The absolute sidelength of a voxel
*/
Eigen::Vector3i Grid3D::point_to_coord(const Eigen::Vector3d &point, const int gridn, const double abs_voxelsize)
{
    int i = 0, j = 0, k = 0;
    float half_gridsize = gridn*abs_voxelsize/2;
    std::cout << "half_gridsize" << half_gridsize << '\n';
    Eigen::Vector3d point_shifted=point.array()+half_gridsize;

    std::cout << "point is " << point_shifted << '\n';

    i=std::floor(point_shifted(0)/abs_voxelsize);
    j=std::floor(point_shifted(1)/abs_voxelsize);
    k=std::floor(point_shifted(2)/abs_voxelsize);


    return Eigen::Vector3i(i, j, k);
}


/**
* \brief Visualize the edges of the grid
*
* \param[in] viewer The igl-viewer handle
* \param[in] color The uniform color for the edges
*/
void Grid3D::visualize_edges(igl::viewer::Viewer &viewer, const Eigen::RowVector3d color)
{
    viewer.data.set_edges(grid_points, grid_edges, color);
}


/**
* \brief Slice the grid at slice index and fill slice with data
*
* what it does:
*   -iterate for a slice index through the other two dimensions and collect the voxel coordinates
*   -map the voxel coordinates to the linear index
*   -query all points inside the respective voxel
*   -accumulate the data inside the voxel (mean)
*   -fill the respective slice entry
*
* \param[in] slice_index The index where we want to slice the grid
* \param[in] dim The dimension we want to slice
* \param[in] data The data we want to see
*/
Eigen::MatrixXd Grid3D::volume_to_slice(const int slice_index, const int dim, const Eigen::VectorXd &data)
{
    assert(dim <= grid_points.cols());
    assert(slice_index <= gridn);

    Eigen::Vector3i coord;
    Eigen::MatrixXd slice_matrix = Eigen::MatrixXd::Zero(gridn, gridn);
    for(int i=0; i < gridn; i++)
    {
        for(int j=0; j < gridn; j++)
        {
            if(dim == 0)
            {
                coord = Eigen::Vector3i(slice_index, i, j);
            }
            else if(dim == 1)
            {
                coord = Eigen::Vector3i(i, slice_index, j);
            }
            else
            {
                coord = Eigen::Vector3i(i, j, slice_index);
            }

            int lin_idx = coord_to_lin_idx(coord, gridn);
            Eigen::VectorXi local_indices = points_indices[lin_idx];

            if(local_indices.size() < 1)
            {
                continue;
            }

            for(int k=0; k < local_indices.size(); k++)
            {
                slice_matrix(i, j) += data(local_indices(k));
            }

            slice_matrix(i, j) /= local_indices.size();
        }
    }
    return slice_matrix;
}

/**
* \brief Insert a slice into the volume grid
*
* what it does:
*   -iterate for a slice index through the other two dimensions and collect the voxel coordinates
*   -map the voxel coordinates to the linear index
*   -fill the volume with the data at lin_idx in the respective dimension (n x 3)
*
* \param[in] slice_index The index where we want to slice the grid
* \param[in] dim The dimension we want to slice
* \param[in] slice The slice we want to insert
* \param[in/out] data The volume where we store the values
*/
void Grid3D::slice_to_volume(const int slice_index, const int dim, const Eigen::MatrixXd &slice,
                             Eigen::MatrixXd &data)
{
    assert(dim <= grid_points.cols());
    assert(slice_index <= gridn);

    Eigen::Vector3i coord;

    for(int i=0; i < gridn; i++)
    {
        for(int j=0; j < gridn; j++)
        {
            if(dim == 0)
            {
                coord = Eigen::Vector3i(slice_index, i, j);
            }
            else if(dim == 1)
            {
                coord = Eigen::Vector3i(i, slice_index, j);
            }
            else
            {
                coord = Eigen::Vector3i(i, j, slice_index);
            }

            int lin_idx = coord_to_lin_idx(coord, gridn);

            data(lin_idx, dim) = slice(i, j);
        }
    }
}

/**
* \brief Smooth the normals with a gaussian kernel with user provided sigma
*
* what it does:
*   -slice the x-dimension of the grid
*   -slice the y-dimension of the grid
*   -slice the z-dimension of the grid
*
*   -smooth the x-values of the normals with a 1D kernel along each fiber of a slice
*   -smooth the y-values of the normals with a 1D kernel along each fiber of a slice
*   -smooth the z-values of the normals with a 1D kernel along each fiber of a slice
*
*   -fill the volume with the smoothed data
*
* TODO:
*   -implement this smoothing
*   -iterate through one dimension
*   -slice dimension x and fill slice with x-values of the normals
*   -slice dimension y and fill slice with y-values of the normals
*   -slice dimension z and fill slice with z-values of the normals
*   -pass the x-slice and the kernel into the conv2d function
*   -pass the y-slice and the kernel into the conv2d function
*   -pass the z-slice and the kernel into the conv2d function
*   -fill the volume (normals_field) with the result of the smoothing
*
* \param[in] normals The normals
* \param[in] sigma The sigma for the gaussian kernel
*/
void Grid3D::smooth_normals(Eigen::MatrixXd &normals, const double sigma) {
    this->normals = normals;

    int kernel_size = 3;
    if (2.0 * 4.0 * std::ceil(sigma / abs_voxelsize) + 1 > 3) {
        kernel_size = 2.0 * 4.0 * std::ceil(sigma / abs_voxelsize) + 1;
    }
    Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(kernel_size, -sigma * 4.0, sigma * 4.0);
    Eigen::MatrixXd kernel = Eigen::MatrixXd::Zero(1, x.size());

    for (int i = 0; i < x.size(); i++) {
        kernel(0, i) = gauss_pdf(x(i), 0, sigma);
    }

    double normalization = kernel.sum();
    kernel /= normalization;

    normals_field.resize(num_voxel, 3);

    // your code here
}

/**
* \brief Compute the normal divergence
*
* what it does:
*   -compute the 6-neighborhood for a voxel
*   -compute the derivative as the central difference between the voxel neigbors
*   -compute the divergence from the derivatives
*
* TODO:
*   -implement this mapping
*   -iterate through (i, j, k) and query the 6 linear indices of the neighbors of a voxel
*   -compute the derivative as central difference along x, y and z directions
*   -compute the divergence from the x, y and z-derivatives and store in tmp_divergence
*   -return the divergence
*/
Eigen::Matrix<short, Eigen::Dynamic, 1> Grid3D::normal_divergence(){

    Eigen::Matrix<short, Eigen::Dynamic, 1> divergence(normals_field.rows());
    Eigen::Matrix<double, Eigen::Dynamic, 1> tmp_divergence(normals_field.rows());

    //TOUCH EXPAND INDEXING DO NOT CHANGE!
    // please use this indexing, otherwise touch expand will not work
    //int lin_index = i + j * gridn + k * gridn * gridn;

    // your code here

    double max = tmp_divergence.maxCoeff();

    for(int i=0; i< num_voxel; i++)
    {
        divergence(i) = static_cast<short>(tmp_divergence(i) / max * 100);
    }

    return divergence;
}
