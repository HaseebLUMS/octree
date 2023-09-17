#include <iostream>
#include <cstring>

#include <quic/QuicConstants.h>
#include "libs/libavif/1.0.1/include/avif/avif.h"

int main(int argc, char * argv[])
{
    const char * outputFilename = "test.avif";

    int returnCode = 1;
    avifEncoder * encoder = NULL;
    avifRWData avifOutput = AVIF_DATA_EMPTY;
    avifRGBImage rgb;
    memset(&rgb, 0, sizeof(rgb));

    avifImage * image = avifImageCreate(128, 128, 8, AVIF_PIXEL_FORMAT_YUV444); // these values dictate what goes into the final AVIF
    if (!image) {
        fprintf(stderr, "Out of memory\n");
        return 1;
    }
    // Configure image here: (see avif/avif.h)
    // * colorPrimaries
    // * transferCharacteristics
    // * matrixCoefficients
    // * avifImageSetProfileICC()
    // * avifImageSetMetadataExif()
    // * avifImageSetMetadataXMP()
    // * yuvRange
    // * alphaPremultiplied
    // * transforms (transformFlags, pasp, clap, irot, imir)

    printf("Encoding from converted RGBA\n");

    avifRGBImageSetDefaults(&rgb, image);
    // Override RGB(A)->YUV(A) defaults here:
    //   depth, format, chromaDownsampling, avoidLibYUV, ignoreAlpha, alphaPremultiplied, etc.

    // Alternative: set rgb.pixels and rgb.rowBytes yourself, which should match your chosen rgb.format
    // Be sure to use uint16_t* instead of uint8_t* for rgb.pixels/rgb.rowBytes if (rgb.depth > 8)
    avifResult allocationResult = avifRGBImageAllocatePixels(&rgb);
    if (allocationResult != AVIF_RESULT_OK) {
        fprintf(stderr, "Allocation of RGB samples failed: %s\n", avifResultToString(allocationResult));
        return 1;
    }

    // Fill your RGB(A) data here
    memset(rgb.pixels, 255, rgb.rowBytes * image->height);

    avifResult convertResult = avifImageRGBToYUV(image, &rgb);
    if (convertResult != AVIF_RESULT_OK) {
        fprintf(stderr, "Failed to convert to YUV(A): %s\n", avifResultToString(convertResult));
        return 1;
    }

    encoder = avifEncoderCreate();
    if (!encoder) {
        fprintf(stderr, "Out of memory\n");
        return 1;
    }
    // Configure your encoder here (see avif/avif.h):
    // * maxThreads
    // * quality
    // * qualityAlpha
    // * tileRowsLog2
    // * tileColsLog2
    // * speed
    // * keyframeInterval
    // * timescale
    encoder->quality = 70;
    encoder->qualityAlpha = AVIF_QUALITY_LOSSLESS;

    // Call avifEncoderAddImage() for each image in your sequence
    // Only set AVIF_ADD_IMAGE_FLAG_SINGLE if you're not encoding a sequence
    // Use avifEncoderAddImageGrid() instead with an array of avifImage* to make a grid image
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

    printf("Encode success: %zu total bytes\n", avifOutput.size);

    FILE * f = fopen(outputFilename, "wb");
    size_t bytesWritten = fwrite(avifOutput.data, 1, avifOutput.size, f);
    fclose(f);
    if (bytesWritten != avifOutput.size) {
        fprintf(stderr, "Failed to write %zu bytes\n", avifOutput.size);
        return 1;
    }
    printf("Wrote: %s\n", outputFilename);

    returnCode = 0;
cleanup:
    if (image) {
        avifImageDestroy(image);
    }
    if (encoder) {
        avifEncoderDestroy(encoder);
    }
    avifRWDataFree(&avifOutput);
    avifRGBImageFreePixels(&rgb); // Only use in conjunction with avifRGBImageAllocatePixels()
    return returnCode;
}