import os.path

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np

# Read in the image
# There are six more images available for reading
# called sample1-6.jpg, feel free to experiment with the others!

filename = 'yellow_ball.jpg'

image = mpimg.imread(filename)


def mask_at_threshold(np_array, threshold):
    '''returns a masked array of either 0 or 1 depending whether a value is above or below threshold'''
    mask = np_array < threshold
    print(mask)
    np_array[mask] = 0 # below threshold
    np_array[~mask] = 1
    return np_array

# Define a function to perform a color threshold
def color_thresh(img, rgb_thresh=(0, 0, 0)):
    ####### TODO 

    color_select = np.zeros_like(img[:,:,0])
    # Create an empty array the same size in x and y as the image 
        # but just a single channel

    mask = (img[:,:,0] > rgb_thresh[0]) & (img[:,:,1] > rgb_thresh[1]) & (img[:,:,2] > rgb_thresh[2])

    color_select[mask] = 1

    # now 
    return color_select

def low_color_thresh(img, rgb_thresh=(0, 0, 0)):
    ####### TODO 

    color_select = np.zeros_like(img[:,:,0])
    # Create an empty array the same size in x and y as the image 
        # but just a single channel

    mask = (img[:,:,0] < rgb_thresh[0]) & (img[:,:,1] < rgb_thresh[1]) & (img[:,:,2] < rgb_thresh[2])

    color_select[mask] = 1

    # now 
    return color_select
    
# Define color selection criteria
###### TODO: MODIFY THESE VARIABLES TO MAKE YOUR COLOR SELECTION

######
lower_threshold = (135,118,12)
upper_threshold = (203,174,36)

# pixels below the thresholds
lower_sel = color_thresh(image, rgb_thresh=lower_threshold)
upper_sel = low_color_thresh(image, rgb_thresh=upper_threshold)
both =  np.all([lower_sel, upper_sel], axis=0)
# Display the original image and binary               
f, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(18, 6), sharey=True)
f.tight_layout()
ax1.imshow(image)
ax1.set_title('Original Image', fontsize=40)

ax2.imshow(lower_sel, cmap='gray')
ax2.set_title('lower threshold', fontsize=40)

ax3.imshow(upper_sel, cmap='gray')
ax3.set_title('upper threshold', fontsize=40)
ax4.imshow(both, cmap='gray')
ax4.set_title('both threshold', fontsize=40)
plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)


plt.show() # Uncomment if running on your local machine
