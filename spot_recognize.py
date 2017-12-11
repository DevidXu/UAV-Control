import cv2
import numpy as np
import random
from skimage import exposure

def spot_detect():
	x=0
	y=0
	center_x=0
	center_y=0
	radius = 0
	is_circle=0
	img = cv2.imread("center2.jpg")
	img = cv2.resize(img, (960, 720))
	#cv2.imshow("ori", img)
	#img = exposure.adjust_log(img)
	#img = exposure.adjust_gamma(img, 2)
	#cv2.imshow("log", img)

	hue_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	blurred = cv2.GaussianBlur(hue_image, (3, 3), 0)

	#low_range = np.array([100, 43, 46])
	low_range = np.array([100, 190, 60])
	#high_range = np.array([124, 255, 255])
	high_range = np.array([115, 255, 255])
	th = cv2.inRange(blurred, low_range, high_range)
	#cv2.imshow('mask', th)

	res=cv2.bitwise_and(blurred,blurred,mask=th)
	#cv2.imshow('result1', res)

	gray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
	#cv2.imshow('gray', gray)

	#dilated = cv2.dilate(gray, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)
	#cv2.imshow('dilated', dilated)

	ret,thresh1 = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
	closed = cv2.morphologyEx(thresh1, cv2.MORPH_CLOSE, kernel)
	dilated = cv2.dilate(closed, kernel)
	eroded = cv2.erode(dilated, kernel)
	cv2.imshow('dilated.jpg', dilated)

	circles = cv2.HoughCircles(dilated, cv2.HOUGH_GRADIENT, 1, len(dilated[1])/20, param1=100, param2=7, minRadius=5, maxRadius=20)

	if circles is not None:
	    x, y, radius = circles[0][0]
	    center = (x, y)
	    cv2.circle(img, center, radius, (0, 255, 0), 2)
	    is_circle = 1
	print(circles[0])
	center_x = img.shape[0]/2
	center_y = img.shape[1]/2
	cv2.imshow('img', img)
	return x, y, center_x, center_y, radius, is_circle

x, y, k, l, r, i = spot_detect()

