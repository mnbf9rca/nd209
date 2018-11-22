import os.path
import cv2

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np

# Read in the image
# There are six more images available for reading
# called sample1-6.jpg, feel free to experiment with the others!

filename = 'color-palette.jpg'

image = mpimg.imread(filename)

    
# Define color selection criteria
###### TODO: MODIFY THESE VARIABLES TO MAKE YOUR COLOR SELECTION

######
lower_threshold = (90,100,190)
upper_threshold = (105,250 ,255)
#low_high_mask = loe_high_color_thresh(image, lower_threshold, upper_threshold)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
# pixels below the thresholds
mask = cv2.inRange(hsv, lower_threshold, upper_threshold)
res = cv2.bitwise_and(image,image, mask= mask)

# Display the original image and binary               
f, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(18, 6), sharey=True)
f.tight_layout()
ax1.imshow(image)
ax1.set_title('Original Image', fontsize=40)

ax2.imshow(hsv)
ax2.set_title('HSV Image', fontsize=40)

ax3.imshow(mask, cmap='gray')
ax3.set_title('mask', fontsize=40)

ax4.imshow(res, cmap='gray')
ax4.set_title('result', fontsize=40)

plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)


plt.show() # Uncomment if running on your local machine
