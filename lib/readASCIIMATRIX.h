// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_READASCIIMATRIX_H
#define IGL_READASCIIMATRIX_H


#include <Eigen/Core>
#include <string>
#include <cstdio>
#include <vector>

namespace igl
{
    // Read a matrix from an ascii file.
    //
    // Templates:
    //   Scalar  type for positions and vectors (will be read as double and cast
    //     to Scalar)
    // Inputs:
    //   filename path to .txt file
    // Outputs:
    //   V  double matrix of data  #V by m
    // Returns true on success, false on errors
    //
    // Example:

    template <typename DerivedV>
    bool readASCIIMATRIX(
            const std::string & filename,
            Eigen::PlainObjectBase<DerivedV> & V);
    // Inputs:
    //   txt_file  pointer to already opened .txt file
    // Outputs:
    //   txt_file  closed file
    template <typename TypeV>
    bool readASCIIMATRIX(
            FILE * txt_file,
            std::vector<std::vector<TypeV> > & V);
    template <typename TypeV>
    bool readASCIIMATRIX(
            const std::string & filename,
            std::vector<std::vector<TypeV> > & V);
}

#include "readASCIIMATRIX.hpp"

#endif

