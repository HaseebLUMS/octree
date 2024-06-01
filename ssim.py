import numpy as np
from PIL import Image
from skimage.metrics import structural_similarity as ssim
import sys

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
    mask1 = extract_non_transparent_regions(image1)
    
    # Extract non-transparent regions
    image1_non_transparent = image1[mask1][:, :3]
    image2_non_transparent = image2[mask1][:, :3]

    # Calculate SSIM on non-transparent regions
    ssim_value = ssim(image1_non_transparent, image2_non_transparent, multichannel=True, win_size=win_size)
    return ssim_value

# Paths to the images
image1_path = sys.argv[1]
image2_path = sys.argv[2]

# Load the images
image1 = load_image(image1_path)
image2 = load_image(image2_path)

# Calculate SSIM
ssim_value = calculate_ssim(image1, image2, win_size=3)
print(f"SSIM: {ssim_value}")
