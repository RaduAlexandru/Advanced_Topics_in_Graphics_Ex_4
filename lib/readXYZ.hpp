// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2017 Alexander Dieckmann <dieckman@cs.uni-bonn.de>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_READXYZ_HPP
#define IGL_READXYZ_HPP

#include "readXYZ.h"
#include <igl/list_to_matrix.h>

#include <iostream>
#include <fstream>
template <typename DerivedV>
inline bool igl::readXYZ(
        const std::string & filename,
        Eigen::PlainObjectBase<DerivedV> & V)
{
    using namespace std;
    vector<vector<typename DerivedV::Scalar> > vV;

    if(!readXYZ(filename,vV))
    {
        return false;
    }

    if(!list_to_matrix(vV,V))
    {
        return false;
    }

    return true;
}

template <typename TypeV>
inline bool igl::readXYZ(
        const std::string & filename,
        std::vector<std::vector<TypeV> > & V)
{
    using namespace std;
    // Should test for ascii

    // Open file, and check for error
    FILE * xyz_file = fopen(filename.c_str(),"r");

    if(NULL==xyz_file)
    {
        fprintf(stderr,"IOError: %s could not be opened...\n",
                filename.c_str());
        return false;
    }
    return readXYZ(xyz_file,V);
}

template <typename TypeV>
inline bool igl::readXYZ(
        FILE * xyz_file,
        std::vector<std::vector<TypeV> > & V)
{
    using namespace std;
    bool is_ascii = true;

    V.clear();

    if(is_ascii) {
        //xyz_file = freopen(NULL, "r", xyz_file);

        if (NULL == xyz_file) {
            fprintf(stderr, "IOError: stl file could not be reopened as ascii ...\n");
            goto close_false;
        }

        //read line by line
        const size_t line_size = 1000;
        char line[line_size];
        int i = 0;
        while(fscanf(xyz_file, "%[^\n]\n", line) != -1) {
            vector<TypeV> v(3);
            char *pch = strstr(line, "LH");
            if (pch){
                //fprintf(stdout, "%s\n", line);
                continue;
            }
            double v1, v2, v3;
            char* token = strtok(line, "\t");
            sscanf(token, "%lg", &v1);
            token = strtok(NULL, "\t");
            sscanf(token, "%lg", &v2);
            token = strtok(NULL, "\t");
            sscanf(token, "%lg", &v3);
            v[0] = v1; v[1] = v2; v[2] = v3;

            V.push_back(v);
        }
        goto close_true;
    }
    close_false:
    fclose(xyz_file);
    return false;
    close_true:
    fclose(xyz_file);
    return true;
}

#endif