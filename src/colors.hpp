#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

#include "libs/jpeg-turbo/2.1.5.1/include/turbojpeg.h"

#include "libs/libwebp-mac/include/webp/types.h"
#include "libs/libwebp-mac/include/webp/encode.h"
#include "libs/libwebp-mac/include/webp/decode.h"

class ColorEncDec {
public:
    virtual int encode(std::vector<uint8_t>& rgb_list, std::vector<uint8_t> &output, int width, int height) = 0;
    virtual void decode(std::vector<uint8_t>& compressed_bytes, std::vector<uint8_t>& decoded_bytes) = 0;
    ColorEncDec(){}
    ~ColorEncDec(){}
protected:
    int nbands_{3};
    int qual_{100};
    std::string test_filename{"./output/test_image.jpg"};
};

class JpegEncDec : public ColorEncDec {
public:
    JpegEncDec() {
        encoder_handle_ = tjInitCompress();
        decoder_handle_ = tjInitDecompress();
        flags_ = 0;
        pixelFormat_ = TJPF_RGB;
        jpegSubsamp_ = TJSAMP_444;
    }

    int encode(std::vector<uint8_t>& rgb_list, std::vector<uint8_t> &output, int width, int height) {
        uint8_t* src_buf = new uint8_t[width * height * nbands_];
        memset(src_buf, 0, width * height * nbands_);
        memcpy(src_buf, &rgb_list[0], rgb_list.size());

        uint8_t* jpeg_buf =NULL;
        unsigned long jpeg_size;
        int tj_stat = tjCompress2(encoder_handle_, src_buf, width, width * nbands_, height, pixelFormat_, 
                                    &(jpeg_buf), &jpeg_size, jpegSubsamp_, qual_, flags_);
        
        if(tj_stat != 0) {
            const char *err = (const char *) tjGetErrorStr();
            std::cerr << "TurboJPEG Error: " << err << " UNABLE TO COMPRESS JPEG IMAGE\n";
            tjDestroy(encoder_handle_);
            encoder_handle_ = NULL;
            return 1;
        }

        output.resize(jpeg_size, 0);
        memcpy(&output[0], jpeg_buf, jpeg_size);

        FILE* qFile= fopen(test_filename.c_str(), "wb");
        fwrite(jpeg_buf, sizeof(uint8_t), jpeg_size, qFile);
        fclose(qFile);

        delete jpeg_buf;
        delete []src_buf;
        return 0;
    }

    void decode(std::vector<uint8_t>& compressed_bytes, std::vector<uint8_t>& decoded_bytes) {
        if(decoder_handle_ == NULL) printf("Can't initialize decoder\n");
        int width = 0;
        int height = 0;
        int jpegSubsamp = 0;
        int jpegColorSpace = 0;

        int num_compressed_bytes = compressed_bytes.size();

        tjDecompressHeader3(decoder_handle_, &compressed_bytes[0], num_compressed_bytes, &width, &height, &jpegSubsamp,
                &jpegColorSpace);

        jpegColorSpace = TJCS_RGB;

        unsigned char *img_buf = (unsigned char *) malloc(width * height * 3);
        if( tjDecompress2(decoder_handle_, &compressed_bytes[0], num_compressed_bytes, img_buf , width, width * 3, height, jpegColorSpace,  0) < 0) {
            printf("JPEG Decompression Failed\n");
            exit(1);
        }

        decoded_bytes.resize(width  *height * 3);
        memcpy(&decoded_bytes[0], img_buf, width * height * 3);
    }

private:
    tjhandle encoder_handle_;
    tjhandle decoder_handle_;
    int flags_;
    int pixelFormat_;
    int jpegSubsamp_;
};

class WebpEncDec : public ColorEncDec {
public:
    WebpEncDec() {}

    int encode(std::vector<uint8_t>& rgb_list, std::vector<uint8_t> &output, int width, int height) {
        uint8_t* src_buf = new uint8_t[width * height * nbands_];
        memset(src_buf, 0, width * height * nbands_);
        memcpy(src_buf, &rgb_list[0], rgb_list.size());

        uint8_t* jpeg_buf =NULL;
        unsigned long jpeg_size;

        jpeg_size = WebPEncodeRGB(src_buf, width, height, 3 * width, 100, &jpeg_buf);
        if (jpeg_size == 0) {
            std::cerr << "WebP Compression Failed." << std::endl;
            WebPFree(&jpeg_buf);
            return 1;
        }

        output.resize(jpeg_size, 0);
        memcpy(&output[0], jpeg_buf, jpeg_size);

        FILE* qFile= fopen(test_filename.c_str(), "wb");
        fwrite(jpeg_buf, sizeof(uint8_t), jpeg_size, qFile);
        fclose(qFile);
        
        delete jpeg_buf;
        delete []src_buf;
        return 0;
    }

    void decode(std::vector<uint8_t>& compressed_bytes, std::vector<uint8_t>& decoded_bytes) {
        int width = 0;
        int height = 0;
        int num_compressed_bytes = compressed_bytes.size();

        WebPGetInfo(&compressed_bytes[0], num_compressed_bytes, &width, &height);
        unsigned char *img_buf = WebPDecodeRGB(&compressed_bytes[0], num_compressed_bytes, &width, &height);

        decoded_bytes.resize(width  *height * 3);
        memcpy(&decoded_bytes[0], img_buf, width * height * 3);
        WebPFree(img_buf);
    }    
};
