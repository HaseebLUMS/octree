from skimage import io, color, util
from skimage.metrics import structural_similarity as ssim
from skimage.filters import threshold_otsu
from skimage.measure import label, regionprops


import sys

if len(sys.argv) < 3:
    print("Provide two images for SSIM calculation")
    exit(1)

im1 = str(sys.argv[1])
im2 = str(sys.argv[2])

def trim_image(image):
    # Convert RGBA to RGB by blending with a white background
    if image.shape[2] == 4:
        image = color.rgba2rgb(image)

    # Convert to grayscale if necessary
    if len(image.shape) == 3:
        image = color.rgb2gray(image)
    
    # Binarize the image using Otsu's thresholding
    thresh = threshold_otsu(image)
    binary = image > thresh
    
    # Label connected components
    label_image = label(binary)
    
    # Find the largest connected component
    props = regionprops(label_image)
    main_props = max(props, key=lambda x: x.area)
    
    # Get the bounding box of the main component
    minr, minc, maxr, maxc = main_props.bbox
    
    # Crop the image to the bounding box
    cropped_image = image[minr:maxr, minc:maxc]
    
    return cropped_image

# Load the two images
image1 = io.imread(im1)
image2 = io.imread(im2)

# Trim the images to only show the main content
image1 = trim_image(image1)
image2 = trim_image(image2)

# Calculate SSIM between the trimmed images
# Assuming the images are in the range [0, 1]
ssim_index, diff = ssim(image1, image2, data_range=image1.max() - image1.min(), full=True)

print(f"SSIM: {ssim_index}")
