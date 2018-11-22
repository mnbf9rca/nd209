import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np

import cv2

# Uncomment the next line for use in a Jupyter notebook
# This enables the interactive matplotlib window
#%matplotlib notebook
image = mpimg.imread('./images/example_grid1.jpg')
plt.subplot(121)
plt.imshow(image)





def perspect_transform(img, src, dst):

    # Get transform matrix using cv2.getPerspectivTransform()
    M = cv2.getPerspectiveTransform(src, dst)
    # Warp image using cv2.warpPerspective()
    # keep same size as input image
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))
    # Return the result
    return warped

# Define source and destination points
source = np.float32([[14.8 ,140.7 ], [ 300.9, 140.7 ], [ 118.5,96.5 ], [ 201.2,96.5 ]])

destination = np.float32([[ (image.shape[1]/2)-5, 150 ], [ (image.shape[1]/2)+5,150 ], [(image.shape[1]/2)-5 , 140 ], [ (image.shape[1]/2)+5,140 ]])      

warped = perspect_transform(image, source, destination)
plt.subplot(122)
plt.imshow(warped)
plt.show() 
