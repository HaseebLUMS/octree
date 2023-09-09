#include "JpegDecoder.hpp"
#include <fstream>
#include <cstdio>

#include "libs/libwebp-mac/include/webp/types.h"
#include "libs/libwebp-mac/include/webp/encode.h"
#include "libs/libwebp-mac/include/webp/decode.h"

JpegDecoder::JpegDecoder() {
}

void JpegDecoder::decode(vector<uint8_t> compressed_bytes, vector<uint8_t>& decoded_bytes) {
    tjhandle handle = tjInitDecompress();
    if(handle == NULL) printf("Can't initialize decoder\n");
    int width = 0;
    int height = 0;
    int jpegSubsamp = 0;
    int jpegColorSpace = 0;

    int numCompressedBytes = compressed_bytes.size();

    WebPGetInfo(&compressed_bytes[0], numCompressedBytes, &width, &height);
    // tjDecompressHeader3(handle, &compressed_bytes[0], numCompressedBytes, &width, &height, &jpegSubsamp,
    //         &jpegColorSpace);
    
    // Not sure why tjDecompressHeader3 is setting jpegcolorspace to 1 instead of 0.
    jpegColorSpace = TJCS_RGB;

    unsigned char *imgBuf = (unsigned char *) malloc(width * height * 3);
    // if( tjDecompress2(handle, &compressed_bytes[0], numCompressedBytes, imgBuf , width, width * 3, height, jpegColorSpace,  0) < 0)
    // {
    //     printf("decompress failed\n");
    //     exit(1);
    // }

    imgBuf = WebPDecodeRGB(&compressed_bytes[0], numCompressedBytes, &width, &height);
    
    decoded_bytes.resize(width  *height * 3);
    memcpy(&decoded_bytes[0], imgBuf, width * height * 3);
}
