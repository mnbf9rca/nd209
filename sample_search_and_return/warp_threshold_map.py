import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
from extra_functions import perspect_transform, color_thresh, source, destination

# Read in the sample image
image = mpimg.imread('sample.jpg')


def rover_coords(binary_img):
    # TODO: fill in this function to
    # Calculate pixel positions with reference to the rover
    # position being at the center bottom of the image.
    '''This is actually the same plot / image as before,
       we're just looking at it with the origin (0, 0) in
       the lower left, rather than the upper left, and the 
       y-axis reversed.

       Now you have x and y pixel positions in image space 
       of the navigable terrain and all you need to do is 
       convert it to rover-centric coordinates. However, you 
       would also like to swap the x and y axes such that 
       they're consistent with the world coordinate system 
       that you'll eventually be mapping to.'''

    ypos, xpos = binary_img.nonzero()

    y_pixel = -(xpos - binary_img.shape[1]/2).astype(np.float)
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Perform warping and color thresholding
warped = perspect_transform(image, source, destination)
colorsel = color_thresh(warped, rgb_thresh=(160, 160, 160))
# Extract x and y positions of navigable terrain pixels
# and convert to rover coordinates


xpix, ypix = rover_coords(colorsel)


plt.subplot(121)
ypos, xpos = colorsel.nonzero()
plt.plot(xpos, ypos, '.')

# Plot the map in rover-centric coords
fig = plt.figure(figsize=(5, 7.5))

plt.subplot(122)
plt.plot(xpix, ypix, '.')
plt.ylim(-160, 160)
plt.xlim(0, 160)
plt.title('Rover-Centric Map', fontsize=20)
plt.show()  # Uncomment if running on your local machine
