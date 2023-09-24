#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

#include <turbojpeg.h>
#include <webp/types.h>
#include <webp/encode.h>
#include <webp/decode.h>
#include <avif/avif.h>

class ColorEncDec {
public:
    virtual int encode(std::vector<uint8_t>& rgb_list, std::vector<uint8_t> &output, int width, int height) = 0;
    virtual void decode(std::vector<uint8_t>& compressed_bytes, std::vector<uint8_t>& decoded_bytes) = 0;
    ColorEncDec(){}
    ~ColorEncDec(){}
protected:
    int nbands_{3};
    int qual_{70};
    std::string test_filename_jpg{"./../output/test_image.jpg"};
    std::string test_filename_webp{"./../output/test_image.webp"};
    std::string test_filename_avif{"./../output/test_image.avif"};
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
        std::cout << "Using Jpeg" << std::endl;
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

        FILE* qFile= fopen(test_filename_jpg.c_str(), "wb");
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
        std::cout << "Using Webp" << std::endl;
        uint8_t* src_buf = new uint8_t[width * height * nbands_];
        memset(src_buf, 0, width * height * nbands_);
        memcpy(src_buf, &rgb_list[0], rgb_list.size());

        uint8_t* jpeg_buf =NULL;
        unsigned long jpeg_size;

        jpeg_size = WebPEncodeRGB(src_buf, width, height, 3 * width, qual_, &jpeg_buf);
        if (jpeg_size == 0) {
            std::cerr << "WebP Compression Failed." << std::endl;
            WebPFree(&jpeg_buf);
            return 1;
        }

        output.resize(jpeg_size, 0);
        memcpy(&output[0], jpeg_buf, jpeg_size);

        FILE* qFile= fopen(test_filename_webp.c_str(), "wb");
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

class AvifEncDec : public ColorEncDec {
public:
    AvifEncDec() {}

    int encode(std::vector<uint8_t>& rgb_list, std::vector<uint8_t> &output, int width, int height) {
        std::cout << "Using Avif" << std::endl;
        avifRWData avifOutput = AVIF_DATA_EMPTY;
        avifRGBImage rgb;
        memset(&rgb, 0, sizeof(rgb));

        avifImage * image = avifImageCreate(width, height, 8, AVIF_PIXEL_FORMAT_YUV444); // these values dictate what goes into the final AVIF
        if (!image) {
            fprintf(stderr, "Out of memory\n");
            return 1;
        }

        avifRGBImageSetDefaults(&rgb, image);
        rgb.rowBytes = width*3;
        rgb.format = AVIF_RGB_FORMAT_RGB;

        avifResult allocationResult = avifRGBImageAllocatePixels(&rgb);
        if (allocationResult != AVIF_RESULT_OK) {
            fprintf(stderr, "Allocation of RGB samples failed: %s\n", avifResultToString(allocationResult));
            return 1;
        }

        memcpy(rgb.pixels, &rgb_list[0], rgb_list.size());
        
        avifResult convertResult = avifImageRGBToYUV(image, &rgb);
        if (convertResult != AVIF_RESULT_OK) {
            fprintf(stderr, "Failed to convert to YUV(A): %s\n", avifResultToString(convertResult));
            return 1;
        }

        avifEncoder * encoder = avifEncoderCreate();
        if (!encoder) {
            fprintf(stderr, "Out of memory\n");
            return 1;
        }
        encoder->quality = 70;
        encoder->qualityAlpha = AVIF_QUALITY_LOSSLESS;

        avifResult addImageResult = avifEncoderAddImage(encoder, image, 1, AVIF_ADD_IMAGE_FLAG_SINGLE);
        if (addImageResult != AVIF_RESULT_OK) {
            fprintf(stderr, "Failed to add image to encoder: %s\n", avifResultToString(addImageResult));
            return 1;
        }

        avifResult finishResult = avifEncoderFinish(encoder, &avifOutput);
        if (finishResult != AVIF_RESULT_OK) {
            fprintf(stderr, "Failed to finish encode: %s\n", avifResultToString(finishResult));
            return 1;
        }

        output.resize(avifOutput.size, 0);
        memcpy(&output[0], avifOutput.data, avifOutput.size);
        printf("Encode success: %zu total bytes\n", avifOutput.size);
        printf("Original: %zu total bytes\n", rgb_list.size());

        FILE * f = fopen(test_filename_avif.c_str(), "wb");
        size_t bytesWritten = fwrite(avifOutput.data, 1, avifOutput.size, f);
        fclose(f);
        if (bytesWritten != avifOutput.size) {
            fprintf(stderr, "Failed to write %zu bytes\n", avifOutput.size);
            return 1;
        }
        return 0;
    }

    void decode(std::vector<uint8_t>& compressed_bytes, std::vector<uint8_t>& decoded_bytes) {
        const char * inputFilename = test_filename_avif.c_str();

        avifRGBImage rgb;
        memset(&rgb, 0, sizeof(rgb));

        avifDecoder * decoder = avifDecoderCreate();
        // Override decoder defaults here (codecChoice, requestedSource, ignoreExif, ignoreXMP, etc)

        avifResult result = avifDecoderSetIOFile(decoder, inputFilename);
        if (result != AVIF_RESULT_OK) {
            fprintf(stderr, "Cannot open file for read: %s\n", inputFilename);
            exit(1);
        }

        result = avifDecoderParse(decoder);
        if (result != AVIF_RESULT_OK) {
            fprintf(stderr, "Failed to decode image: %s\n", avifResultToString(result));
            exit(1);
        }

        // Now available:
        // * All decoder->image information other than pixel data:
        //   * width, height, depth
        //   * transformations (pasp, clap, irot, imir)
        //   * color profile (icc, CICP)
        //   * metadata (Exif, XMP)
        // * decoder->alphaPresent
        // * number of total images in the AVIF (decoder->imageCount)
        // * overall image sequence timing (including per-frame timing with avifDecoderNthImageTiming())

        printf("Parsed AVIF: %ux%u (%ubpc)\n", decoder->image->width, decoder->image->height, decoder->image->depth);

        while (avifDecoderNextImage(decoder) == AVIF_RESULT_OK) {
            std::cout << "Image++" << std::endl;
            // Now available (for this frame):
            // * All decoder->image YUV pixel data (yuvFormat, yuvPlanes, yuvRange, yuvChromaSamplePosition, yuvRowBytes)
            // * decoder->image alpha data (alphaPlane, alphaRowBytes)
            // * this frame's sequence timing

            avifRGBImageSetDefaults(&rgb, decoder->image);
            rgb.format = AVIF_RGB_FORMAT_RGB;
            // Override YUV(A)->RGB(A) defaults here:
            //   depth, format, chromaUpsampling, avoidLibYUV, ignoreAlpha, alphaPremultiplied, etc.

            // Alternative: set rgb.pixels and rgb.rowBytes yourself, which should match your chosen rgb.format
            // Be sure to use uint16_t* instead of uint8_t* for rgb.pixels/rgb.rowBytes if (rgb.depth > 8)
            result = avifRGBImageAllocatePixels(&rgb);
            if (result != AVIF_RESULT_OK) {
                fprintf(stderr, "Allocation of RGB samples failed: %s (%s)\n", inputFilename, avifResultToString(result));
                exit(1);
            }

            result = avifImageYUVToRGB(decoder->image, &rgb);
            if (result != AVIF_RESULT_OK) {
                fprintf(stderr, "Conversion from YUV failed: %s (%s)\n", inputFilename, avifResultToString(result));
                exit(1);
            }

            decoded_bytes.resize(rgb.width*rgb.height*3);
            memcpy(&decoded_bytes[0], rgb.pixels, rgb.width*rgb.height*3);
        }

        return;
    }
};