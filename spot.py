from parameters import *
from skimage import exposure
from Feature import *
import Region, Camera
import threading
import cv2,time,os,math
'''
I.  You need to complete two functions: position, ideal
II. In position(), you need to apply your algorithm, if no result or no existence, you should append
    (None..) to self.pos, else you append your result.
    (Pay attention: There is a check in Feature.py to run your position function twice for a single 
    image. If these positions are seperated seriously, the result will fail)
III.In ideal(), you should judge whether the current image is ideal so that we can amplify the camera to 
    get high-resolution image to give you for further process.
IV. You could add your class variables as you want
V.  If you want to print information, use logging:
        self.log.info("name: message")
        self.log.warn("name: message")
    The screen will only show warning information.
'''

'''
This is the class of spot detection which is similar to the one for license plate
'''
class Spot(feature):
    # self-defined parameters
    x = 0
    y = 0
    center_x = 0
    center_y = 0
    radius = 0
    phase = 0 # 0 means no spot detected, 1 right, 2 left
    found = False

    def __init__(self,name='plate',num_threads=3,position=[0.0,0.0,0.0], log=None):
        feature.__init__(self, num_threads, \
                         Region.region(position=position, tar_pos=[SR_LENGTH/2+0.3,1.0,SR_WIDTH/2+0.5], radius=[0.1,0.1,0.1]), \
                         Camera.camera(angle=[-25.0,0.0]), name, log)
        self.ret.name=name
        self.num_threads=1
        self.log=log
        self.phase = 1
        # define your variables
        pass

    # pay attention: before we call this function, self.pos should be set to empty
    def position(self, image, arg0=None, arg1=None, arg2=None):
        # cv2.imwrite("./modules/spot/images/spot.jpg", image)
        self.x, self.y, self.center_x, self.center_y, self.radius, is_circle = spot_detect(image)
        if self.phase == 1:
            if is_circle == 1:
                self.ret.exist = True
                self.pos.append([self.x-2*self.radius,self.y-2*self.radius,self.x+2*self.radius,self.y+2*self.radius])
                part_image = image[self.y-2*self.radius:self.y+2*self.radius,self.x-2*self.radius:self.x+2*self.radius]
                cv2.imwrite("spot.jpg",part_image)
                self.ret.image = cv2.imread("spot.jpg")
                self.log.info("Spot feature found on right side.")
            else:
                self.phase = 2
                self.region.tar_pos = [SR_LENGTH/2,2.0,-SR_WIDTH/2-0.5]
                self.log.info("Need to go to another side of the car.")
        elif self.phase == 2:
            #UAV fly to the left side of the car
            if is_circle == 1:
                self.ret.exist = True
                self.pos.append([self.x-2*self.radius,self.y-2*self.radius,self.x+2*self.radius,self.y+2*self.radius])
            else:
                self.phase = 1
                self.log.info("Cannot find spot feature on both sides")

        return
            


    # before we call it, make sure you have something in the self.pos array
    def ideal(self, image, rect):
        # define your method
        width=image.shape[0]
        height=image.shape[1]
        channels=image.shape[2]
        try:
            assert(len(self.pos)>0)
        except:
            self.log.info("The length of detected position is 0. No way to test if ideal")
            return False

        if 1.0*(rect[0]+rect[2]/2-width/2)/width>PLATE_CENTER_BIAS or \
                                        1.0*(rect[1]+rect[3]/2-height/2)/height>PLATE_CENTER_BIAS:
            self.log.info("Plate Center Bias is unacceptable to be ideal!")
            return False
        if 1.0*rect[2]/width<PLATE_PHOTO_RATE or 1.0*rect[3]/height<PLATE_PHOTO_RATE:
            self.log.info("Plate photo accounts too small part in the image!")
            return False

        return True

    # no need to change
    def multi_detect(self,image):
        self.pos, threads=[],[]
        self.position(image)

    # should return [relative_region_position, relative_camera_position]
    def translate(self, image, ret, region, camera):
        distance_hor = abs(self.x-self.center_x)
        distance_ver = abs(self.y-self.center_y)
        distance_hor = distance_hor / image.shape[0] #width rate
        distance_ver = distance_ver / image.shape[1] #height rate
        #actual distance = distance_hor * parameter
        move_hor = 0
        move_ver = 0
        if self.phase == 1: #at right side
            move_hor = self.x - self.center_x
            move_ver = -(self.y - self.center_y)
        if self.phase == 2: #at left side
            move_hor = -(self.x - self.center_x)
            move_ver = -(self.y - self.center_y)

        move_hor *= 1.0 * HOR_RATE
        move_ver *= 1.0 * VER_RATE
        move_hor = move_hor / image.shape[0]
        move_ver = move_ver / image.shape[1]
        movement = [move_hor, move_ver, 0] # this is because image coordinate is different
        for i in range(3):
            movement[i]+=region.center[i]
            movement[i]=float('%.2f' % movement[i])
        return [movement, camera.angle+[0.0,0.0]]

    def postprocess(self, image, UAV):
        UAV.log.info("Spot detected and going forward for better image!")
        tar_pos = UAV.region.center[:]
        tar_pos[2]-=0.3
        tar_pos[0]+=0.1
        UAV.move(tar_pos, self.camera.angle)
        self.image = UAV.comm.waitImage(COMPRESS)
        UAV.gui.centerImageQueue.append(self.image)
        tar_pos[2]+=0.3
        tar_pos[0]-=0.1
        UAV.move(tar_pos, self.camera.angle)
        return

def spot_detect(img):
	x=0
	y=0
	center_x=0
	center_y=0
	radius = 0
	is_circle=0
	# cv2.imwrite("ori.jpg", img)
	#img = exposure.adjust_log(img)
	#img = exposure.adjust_gamma(img, 2)
	#cv2.imshow("log", img)

	hue_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	blurred = cv2.GaussianBlur(hue_image, (3, 3), 0)
	# cv2.imwrite("blur.jpg",blurred)


	#low_range = np.array([100, 43, 46])
	low_range = np.array([100, 160, 60])
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
	# cv2.imwrite('dilated.jpg', dilated)

	circles = cv2.HoughCircles(dilated, cv2.HOUGH_GRADIENT, 1, len(dilated[1])/20, param1=100, param2=7, minRadius=5, maxRadius=20)

	if circles is not None:
	    x, y, radius = circles[0][0]
	    center = (x, y)
	    cv2.circle(img, center, radius, (0, 255, 0), 2)
	    is_circle = 1
	'''
	for i in circles[0]:
		x, y, radius = circles[0][i]
		print x, y
	'''
	print(circles)
	center_x = img.shape[0]/2
	center_y = img.shape[1]/2
	# cv2.imwrite('img.jpg', img)
	return int(x), int(y), int(center_x), int(center_y), int(radius), is_circle


if __name__=="__main__":
    test = Spot()
    image = cv2.imread("center2.jpg")
    test.position(image)

