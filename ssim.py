import numpy as np
from PIL import Image
from skimage.metrics import structural_similarity as ssim
import sys
import math

def load_image(image_path):
    """Load an image from a file path and return it as a numpy array."""
    image = Image.open(image_path).convert('RGBA')
    return np.array(image)

def extract_non_transparent_regions(image_array):
    """Extract non-transparent regions of an image based on the alpha channel."""
    alpha_channel = image_array[:, :, 3]
    non_transparent_mask = alpha_channel > 0
    return non_transparent_mask

def calculate_ssim(image1, image2, win_size=7):
    """Calculate SSIM between two images ignoring transparent background."""

    # Calculate SSIM on non-transparent regions
    ssim_value = ssim(image1, image2, multichannel=True, win_size=win_size)
    return ssim_value

def calculate_psnr(image1, image2):
    """Calculate Peak Signal-to-Noise Ratio (PSNR) between two images."""

    mse = np.mean((image1 - image2) ** 2)
    max_pixel = 255.0
    psnr = 20 * math.log10(max_pixel / math.sqrt(mse))
    return psnr

# Paths to the images
image1_path = sys.argv[1]
image2_path = sys.argv[2]

# Load the images
image1 = load_image(image1_path)
image2 = load_image(image2_path)

mask = extract_non_transparent_regions(image1)

# Extract non-transparent regions
image1_non_transparent = image1[mask][:, :3]  # get rid of alpha channel
image2_non_transparent = image2[mask][:, :3]

# Calculate SSIM
ssim_value = calculate_ssim(image1_non_transparent, image2_non_transparent, win_size=3)
print(f"SSIM: {ssim_value}")

psnr_value = calculate_psnr(image1_non_transparent, image2_non_transparent)
print(f"PSNR: {psnr_value}")
