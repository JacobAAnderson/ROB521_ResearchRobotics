###############################
#######BeanBags#################
###############################

import numpy as np
import cv2

# load image
cap = cv2.VideoCapture("imgs/WIN_20190425_17_18_32_Pro.mp4")

params = cv2.SimpleBlobDetector_Params()

while(True):
    # Capture frame-by-frame
        ret, img = cap.read()
        # add blur because of pixel artefacts 
        img = cv2.GaussianBlur(img, (5, 5),5)
        # convert to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 
        # set lower and upper color limits
        lower_blue = np.array([104, 232, 91])
        upper_blue = np.array([180,255,255])
        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, (104, 232, 91), (180,255,255))
        mask_red = cv2.inRange(hsv, (161, 128, 0), (180,255,255))
        final_mask = mask + mask_red
        # apply mask to original image
        res = cv2.bitwise_and(img,img, mask= final_mask)
        #show imag
        #cv2.imshow("Result", res)
        # detect contours in image
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # draw filled contour on result
                #for cnt in contours:
                        #cv2.drawContours(res, [cnt], 0, (0,0,255), 2)
        # detect edges in mask
        edges = cv2.Canny(mask,100,100)
        # to save an image use cv2.imwrite('filename.png',img)  
        #show images
        #cv2.imshow("Result_with_contours", res)
        #cv2.imshow("Mask", mask)
        #cv2.imshow("Edges", edges)



        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 255
        
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 75
        params.maxArea = 20000
        
        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.01
        
        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.01
        
        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01
        # Create a detector with the parameters
        detector = cv2.SimpleBlobDetector_create(params)
        # Detect blobs.

        inverted_img = cv2.bitwise_not(res)
        keypoints = detector.detect(inverted_img)
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
        # the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(res, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #location of blobs
        #x = keypoints[i].pt[0] #i is the index of the blob you want to get the position
        #y = keypoints[i].pt[1]
        red = [0,0,255]
        for keyPoint in keypoints:
                x = keyPoint.pt[0]
                y = keyPoint.pt[1]
                cv2.circle(im_with_keypoints, (int(x),int(y)),10, (0,0,255),-1, 1,0);
        # Show blobs
        im_with_keypoints = cv2.resize(im_with_keypoints, (806, 605))
        cv2.imshow("Keypoints", im_with_keypoints)


        if cv2.waitKey(1) & 0xFF == ord('q'):
                break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()