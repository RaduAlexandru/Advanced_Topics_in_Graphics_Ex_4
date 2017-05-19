// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_READXYZ_H
#define IGL_READXYZ_H


#include <Eigen/Core>
#include <string>
#include <cstdio>
#include <vector>

namespace igl
{
    // Read a mesh from an ascii file.
    //
    // Templates:
    //   Scalar  type for positions and vectors (will be read as double and cast
    //     to Scalar)
    // Inputs:
    //   filename path to .xyz file
    // Outputs:
    //   V  double matrix of vertex positions  #V*3 by 3
    // Returns true on success, false on errors
    //
    // Example:
    //   bool success = readXYZ(filename,temp_V,F);
    //   remove_duplicate_vertices(temp_V,0,V,SVI,SVJ);
    //   for_each(F.data(),F.data()+F.size(),[&SVJ](int & f){f=SVJ(f);});
    //   writeOBJ("Downloads/cat.obj",V,F);
    template <typename DerivedV>
    bool readXYZ(
            const std::string & filename,
            Eigen::PlainObjectBase<DerivedV> & V);
    // Inputs:
    //   xyz_file  pointer to already opened .xyz file
    // Outputs:
    //   stl_file  closed file
    template <typename TypeV>
    bool readXYZ(
            FILE * xyz_file,
            std::vector<std::vector<TypeV> > & V);
    template <typename TypeV>
    bool readXYZ(
            const std::string & filename,
            std::vector<std::vector<TypeV> > & V);
}

#include "readXYZ.hpp"

#endif

