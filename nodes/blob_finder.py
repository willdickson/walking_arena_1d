from __future__ import print_function
import cv2
import numpy

class BlobFinder(object):

    def __init__(self, threshold=200, filterByArea=True, minArea=100, maxArea=None):
        self.threshold = threshold
        self.filterByArea = filterByArea 
        self.minArea = minArea 
        self.maxArea = maxArea 

    def find(self,image):
        
        rval, threshImage = cv2.threshold(image, self.threshold,255,cv2.THRESH_BINARY)
        _, contourList, _ = cv2.findContours(threshImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find blob data
        blobList = []
        blobContours = []

        for contour in contourList:

            blobOk = True

            # Get area and apply area filter  
            area = cv2.contourArea(contour)
            if self.filterByArea:
                if area <= 0:
                    blobOk = False
                if self.minArea is not None:
                    if area < self.minArea:
                        blobOk = False
                if self.maxArea is not None:
                    if area > self.maxArea:
                        blobOk = False

            # Get centroid
            moments = cv2.moments(contour)
            if moments['m00'] > 0 and blobOk:
                centroidX = int(numpy.round(moments['m10']/moments['m00']))
                centroidY = int(numpy.round(moments['m01']/moments['m00']))
            else:
                blobOk = False
                centroidX = 0
                centroidY = 0

            # Get bounding rectangle
            if blobOk:
                bound_rect = cv2.boundingRect(contour)
                minX = bound_rect[0]
                minY = bound_rect[1]
                maxX = bound_rect[0] + bound_rect[2] 
                maxY = bound_rect[1] + bound_rect[3] 
            else:
                minX = 0.0 
                minY = 0.0
                maxX = 0.0
                maxY = 0.0

            # Create blob dictionary
            blob = {
                    'centroidX' : centroidX,
                    'centroidY' : centroidY,
                    'minX'      : minX,
                    'maxX'      : maxX,
                    'minY'      : minY,
                    'maxY'      : maxY,
                    'area'      : area,
                    } 

            # If blob is OK add to list of blobs
            if blobOk: 
                blobList.append(blob)
                blobContours.append(contour)

        # Draw blob on image
        blobImage = cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
        cv2.drawContours(blobImage,blobContours,-1,(0,0,255),3)

        return blobList, blobImage






