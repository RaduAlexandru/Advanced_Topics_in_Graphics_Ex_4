// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2017 Alexander Dieckmann <dieckman@cs.uni-bonn.de>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_READASCIIMATRIX_HPP
#define IGL_READASCIIMATRIX_HPP

#include "readASCIIMATRIX.h"
#include <igl/list_to_matrix.h>

#include <iostream>
#include <fstream>
template <typename DerivedV>
inline bool igl::readASCIIMATRIX(
        const std::string & filename,
        Eigen::PlainObjectBase<DerivedV> & V)
{
    using namespace std;
    vector<vector<typename DerivedV::Scalar> > vV;

    if(!readASCIIMATRIX(filename,vV))
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
inline bool igl::readASCIIMATRIX(
        const std::string & filename,
        std::vector<std::vector<TypeV> > & V)
{
    using namespace std;
    // Should test for ascii

    // Open file, and check for error
    FILE * txt_file = fopen(filename.c_str(),"r");

    if(NULL==txt_file)
    {
        fprintf(stderr,"IOError: %s could not be opened...\n",
                filename.c_str());
        return false;
    }
    return readASCIIMATRIX(txt_file,V);
}

template <typename TypeV>
inline bool igl::readASCIIMATRIX(
        FILE * txt_file,
        std::vector<std::vector<TypeV> > & V)
{
    using namespace std;
    bool is_ascii = true;

    V.clear();

    if(is_ascii) {
        //txt_file = freopen(NULL, "r", txt_file);

        if (NULL == txt_file) {
            fprintf(stderr, "IOError: stl file could not be reopened as ascii ...\n");
            goto close_false;
        }

        //read line by line
        const size_t line_size = 10000;
        char line[line_size];
        int i = 0;
        while(fscanf(txt_file, "%[^\n]\n", line) != -1) {
            vector<TypeV> v;
            char* token = strtok(line, ",");
            double _v;

            sscanf(token, "%lg", &_v);
            v.push_back(_v);

            int elements_per_row = 0;
            while((token = strtok(NULL, ",")) != NULL){
                sscanf(token, "%lg", &_v);
                v.push_back(_v);
                elements_per_row++;
            }

            V.push_back(v);
        }
        goto close_true;
    }
    close_false:
    fclose(txt_file);
    return false;
    close_true:
    fclose(txt_file);
    return true;
}

#endif