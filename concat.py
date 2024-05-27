import numpy as np
from PIL import Image

def concatenate_images(image_list):
    # Determine the maximum height among the images
    max_height = max(img.shape[0] for img in image_list)
    
    # Stack images horizontally
    concatenated_image = np.hstack([
        np.pad(img, ((0, max_height - img.shape[0]), (0, 0), (0, 0)), mode='constant')
        for img in image_list
    ])
    
    return concatenated_image

# Example usage:
# Assuming image_list contains the list of images (as NumPy arrays)
# Replace this with your list of images
# image_list = [np.array(Image.open(f'image_{i}.png')) for i in range(1, 6)]

image_list = ["./output/ricardo.ply/000.png", "./output/ricardo.ply/1000.png"]
image_list = [np.array(Image.open(ele)) for ele in image_list]
# Concatenate images
concatenated_image = concatenate_images(image_list)

# Save or display the concatenated image
concatenated_image_pil = Image.fromarray(concatenated_image)
concatenated_image_pil.save('concatenated_image.png')
concatenated_image_pil.show()
