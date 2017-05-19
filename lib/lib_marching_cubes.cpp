#include "lib_marching_cubes.h"
#include "marching_cubes_table.h"
#include <Eigen/Core>

double hearts_function(const Eigen::VectorXd &points){
    double signed_distance;
    double x = points(0);
    double y = points(1);
    double z = points(2);
    return std::pow(x*x + 9. / 4. * y*y + z*z - 1, 3)- x*x * z*z*z - 9. / 80. * y*y * z*z*z;
}

Eigen::Vector3i lin_idx_to_coord(const int n, const int gridn)
{
    // n = i + j* gridn + k * gridn*gridn
    int i = n % gridn;
    int j = ((n - i) / gridn) % gridn;
    int k = ((n - i - j * gridn) / gridn) / gridn;
    return Eigen::Vector3i(i, j, k);
}

int coord_to_lin_idx(const Eigen::Vector3i &coord, const int gridn)
{
    return coord(0) + coord(1) * gridn + coord(2) * gridn * gridn;
}

Eigen::Vector3d coord_to_voxel_center(const Eigen::Vector3i &coord, const int gridn, const double abs_voxelsize)
{
    double half_gridsize = (double)gridn * abs_voxelsize / 2.0;
    double x = ((double)coord(0) + 0.5) * abs_voxelsize - half_gridsize;
    double y = ((double)coord(1) + 0.5) * abs_voxelsize - half_gridsize;
    double z = ((double)coord(2) + 0.5) * abs_voxelsize - half_gridsize;
    return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3i point_to_coord(const Eigen::Vector3d &point, const int gridn, const double abs_voxelsize)
{
    double half_gridsize = (double)gridn * abs_voxelsize / 2.0;
    int i = floor((point(0) + half_gridsize) / abs_voxelsize);
    int j = floor((point(1) + half_gridsize) / abs_voxelsize);
    int k = floor((point(2) + half_gridsize) / abs_voxelsize);
    return Eigen::Vector3i(i, j, k);
}


void construct_sdf_field(const int gridn, const double abs_voxelsize, Eigen::VectorXd &sdf_field)
{
    for(int i=0; i < sdf_field.size(); i++)
    {
        Eigen::Vector3d center = coord_to_voxel_center(lin_idx_to_coord(i, gridn), gridn, abs_voxelsize);
        sdf_field(i) = hearts_function(center);
        //sdf_field(i) = cube_function(center, 2.0, 2.0, 2.0);
    }
}

Eigen::Vector3d interpolate(const double isovalue, const Eigen::Vector3d &p1, const double sdf1, const Eigen::Vector3d &p2, double sdf2)
{
    if(fabs(sdf1 - sdf2) < 1e-16){
        return p1;
    }

    double t = (isovalue - sdf1) / (sdf2 - sdf1);
    return (1 - t) * p1 + t * p2;
}

std::vector<Triangle> marching_cubes(const Eigen::VectorXd &sdf_field, const int gridn, const double abs_voxelsize,
                                     const double isovalue)
{
    double I = abs_voxelsize;
    double O = 0.0;
    std::vector<Triangle> triangles;

    for(int i=0; i < sdf_field.size(); i++)
    {
        Eigen::Vector3d center = coord_to_voxel_center(lin_idx_to_coord(i, gridn), gridn, abs_voxelsize);
        Eigen::Vector3d voxel[8] =
                {
                        center + Eigen::Vector3d(O, O, O),
                        center + Eigen::Vector3d(I, O, O),
                        center + Eigen::Vector3d(I, I, O),
                        center + Eigen::Vector3d(O, I, O),
                        center + Eigen::Vector3d(O, O, I),
                        center + Eigen::Vector3d(I, O, I),
                        center + Eigen::Vector3d(I, I, I),
                        center + Eigen::Vector3d(O, I, I)
                };

        bool isValid = true;
        double sdf[8];
        for(int j=0; j < 8; j++)
        {
            int lin_idx = coord_to_lin_idx(point_to_coord(voxel[j], gridn, abs_voxelsize), gridn);

            if (lin_idx >= gridn * gridn * gridn){
                isValid = false;
            }else{
                sdf[j] = sdf_field[lin_idx];
            }
        }

        if(!isValid)
        {
            continue;
        }

        unsigned int cubeindex = 0;
        for(int j=0; j < 8; j++)
        {
            if(sdf[j] > isovalue)
            {
                cubeindex |= (1 << j);
            }
        }

        if(edge_table[cubeindex] == 0)
        {
            continue;
        }

        Eigen::Vector3d vertex_list[12];
        if (edge_table[cubeindex] & (1 << 0))  { vertex_list[0]  = interpolate(isovalue, voxel[0], sdf[0], voxel[1], sdf[1]);}
        if (edge_table[cubeindex] & (1 << 1))  { vertex_list[1]  = interpolate(isovalue, voxel[1], sdf[1], voxel[2], sdf[2]);}
        if (edge_table[cubeindex] & (1 << 2))  { vertex_list[2]  = interpolate(isovalue, voxel[2], sdf[2], voxel[3], sdf[3]);}
        if (edge_table[cubeindex] & (1 << 3))  { vertex_list[3]  = interpolate(isovalue, voxel[3], sdf[3], voxel[0], sdf[0]);}
        if (edge_table[cubeindex] & (1 << 4))  { vertex_list[4]  = interpolate(isovalue, voxel[4], sdf[4], voxel[5], sdf[5]);}
        if (edge_table[cubeindex] & (1 << 5))  { vertex_list[5]  = interpolate(isovalue, voxel[5], sdf[5], voxel[6], sdf[6]);}
        if (edge_table[cubeindex] & (1 << 6))  { vertex_list[6]  = interpolate(isovalue, voxel[6], sdf[6], voxel[7], sdf[7]);}
        if (edge_table[cubeindex] & (1 << 7))  { vertex_list[7]  = interpolate(isovalue, voxel[7], sdf[7], voxel[4], sdf[4]);}
        if (edge_table[cubeindex] & (1 << 8))  { vertex_list[8]  = interpolate(isovalue, voxel[0], sdf[0], voxel[4], sdf[4]);}
        if (edge_table[cubeindex] & (1 << 9))  { vertex_list[9]  = interpolate(isovalue, voxel[1], sdf[1], voxel[5], sdf[5]);}
        if (edge_table[cubeindex] & (1 << 10)) { vertex_list[10] = interpolate(isovalue, voxel[2], sdf[2], voxel[6], sdf[6]);}
        if (edge_table[cubeindex] & (1 << 11)) { vertex_list[11] = interpolate(isovalue, voxel[3], sdf[3], voxel[7], sdf[7]);}

        for (int j = 0; triangle_table[cubeindex][j] != -1; j += 3)
        {
            Triangle triangle;

            for (int k = 0; k < 3; ++k)
            {
                triangle.vertices[k] = vertex_list[triangle_table[cubeindex][j + k]];
            }

            triangles.push_back(triangle);
        }
    }
    return triangles;
}