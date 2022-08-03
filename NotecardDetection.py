import numpy as np
import cv2
from PIL import Image
import os

path = '/Users/taylorkorte/Desktop/Raytheon/RPi Code/CameraPics'

for i,file in enumerate(os.listdir(path)): 

	filename = file.split(".")[0]
	img_path = os.path.join(path, file)
	img = Image.open(img_path)
	img = cv2.imread(img_path, cv2.IMREAD_COLOR)

	# Convert the imageFrame in BGR(RGB color space) to
	# HSV(hue-saturation-value) color space
	hsvFrame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	# Set range for red and define mask
	red_lower = np.array([136, 87, 111], np.uint8)
	red_upper = np.array([180, 255, 255], np.uint8)
	red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

	# Set range for green and define mask
	green_lower = np.array([70, 80, 68], np.uint8)
	green_upper = np.array([150, 176, 135], np.uint8)
	green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

	# Set range for yellow and define mask
	yellow_lower = np.array([180, 210, 215], np.uint8)
	yellow_upper = np.array([220, 250, 250], np.uint8)
	yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)

	# Morphological Transform, Dilation and bitwise_and operator
	# between imageFrame and mask 
	kernel = np.ones((5, 5), "uint8")

	red_mask = cv2.dilate(red_mask, kernel)
	res_red = cv2.bitwise_and(img, img,
							mask = red_mask)

	green_mask = cv2.dilate(green_mask, kernel)
	res_green = cv2.bitwise_and(img, img,
								mask = green_mask)

	yellow_mask = cv2.dilate(yellow_mask, kernel)
	res_yellow = cv2.bitwise_and(img, img,
							mask = yellow_mask)

	# # Creating contour to track red color
	# contours_red, hierarchy_red = cv2.findContours(red_mask,
	# 									cv2.RETR_TREE,
	# 									cv2.CHAIN_APPROX_SIMPLE)
	# areas_red = []
	# for contour_red in contours_red:
	# 	areas_red.append(cv2.contourArea(contour_red))

	# contour_red = contours_red[areas_red.index(max(areas_red))]
	# area_red = cv2.contourArea(contour_red)

	# x_red, y_red, w_red, h_red = cv2.boundingRect(contour_red)

	# imageFrame_red = cv2.rectangle(img, (x_red, y_red),
	# 						(x_red + w_red, y_red + h_red),
	# 						(0, 0, 255), 2)

	# cv2.putText(img, "Red", (x_red, y_red),
	# 			cv2.FONT_HERSHEY_SIMPLEX, 1.0,
	# 			(0, 0, 255))	

	# Creating contour to track green color
	# contours_green, hierarchy_green = cv2.findContours(green_mask,
	# 									cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	# areas_green = [cv2.contourArea(contour_green) for contour_green in contours_green]

	# contour_green = contours_green[areas_green.index(max(areas_green))]
	# area_green= cv2.contourArea(contour_green)

	# x_green, y_green, w_green, h_green = cv2.boundingRect(contour_green)

	# imageFrame_green = cv2.rectangle(img, (x_green, y_green),
	# 						(x_green + w_green, y_green + h_green),
	# 						(0, 255, 0), 2)

	# cv2.putText(img, "Green", (x_green, y_green),
	# 			cv2.FONT_HERSHEY_SIMPLEX,
	# 			1.0, (0, 255, 0))



	contours_yellow, hierarchy_yellow = cv2.findContours(yellow_mask,
										cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	areas_yellow = [cv2.contourArea(contour_yellow) for contour_yellow in contours_yellow]


	contour_yellow = contours_yellow[areas_yellow.index(max(areas_yellow))]
	area_yellow = cv2.contourArea(contour_yellow)

	x_yellow, y_yellow, w_yellow, h_yellow = cv2.boundingRect(contour_yellow)
	imageFrame_yellow = cv2.rectangle(img, (x_yellow, y_yellow),
							(x_yellow + w_yellow, y_yellow + h_yellow),
							(255, 255, 255), 5)
			
	cv2.putText(img, "Yellow", (x_yellow, y_yellow),
				cv2.FONT_HERSHEY_SIMPLEX,
				1.0, (255, 255, 255))
			
	# Program Termination
	cv2.imshow("Result Image", img)

	if cv2.waitKey(0) & 0xff == 27:
	    cv2.destroyAllWindows()


