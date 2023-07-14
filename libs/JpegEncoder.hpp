#pragma once

#include <cstdlib>
#include <iostream>
#include <cstdio>
#include <vector>
#include <cstring>

#include "libs/jpeg-turbo/2.1.5.1/include/turbojpeg.h"

using namespace std;

class JpegEncoder{
public:
    JpegEncoder();
    ~JpegEncoder(){ printf("[JPEG] encoder deleted\n");};

    int encode(vector<uint8_t> rgb_list, vector<uint8_t> &jpeg, int width, int height);
private:
    tjhandle handle_;
    int jpegQual_;
    int nbands_;
    int flags_;
    int pixelFormat_;
    int jpegSubsamp_;

};


