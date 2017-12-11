from Feature import *
import threading
import Region
import Camera

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


class Ancheck(feature):
    leftWidth = 2
    rightWidth = 18
    upHeight = 2
    lowHeight = 18
    # these initial parameters should be modified according to tests
    moveRight = 0
    moveUp = 0
    record = []
    returnImage = None

    def __init__(self, name='Ancheck', num_threads=3, position=[0.0, 0.0, 0.0], log=None):
        feature.__init__(self, num_threads,
                         Region.region(position=position, tar_pos=[2.0, 4.0, 0.0], radius=[0.1, 0.1, 0.1]),
                         Camera.camera(angle=[65.0, -80.0]), name, log)
        self.ret.name = name
        self.num_threads = 1
        self.log = log
        imgRs = []
        # define your variables
        pass

    def position(self, image):
        # define your method
        # judge whether the feature exist
        sh = image.shape
        # variables to determine a area to detect the marks

        while True:
            option = 9
            a, b, c, d = sh[0] / 20 * self.upHeight, sh[0] / 20 * self.lowHeight, sh[1] / 20 * self.leftWidth, sh[
                1] / 20 * self.rightWidth
            img = image[int(a): int(b), int(c): int(d)]
            self.returnImage = img.copy()

            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # cv2.imshow("hsv",hsv)

            img = cv2.GaussianBlur(img, (5, 5), 0)
            # cv2.imshow("hsv",hsv)

            if (option == 0):  # blue
                lower = np.array([100, 43, 46])
                upper = np.array([124, 255, 255])
            elif (option == 1):  # yellow
                lower = np.array([26, 43, 46])
                upper = np.array([34, 255, 255])
            elif (option == 2):
                lower = np.array([0, 0, 0])
                upper = np.array([180, 255, 46])
            else:  # green
                lower = np.array([75, 43, 46])
                upper = np.array([92, 255, 255])

            mask = cv2.inRange(hsv, lower, upper)
            res = cv2.bitwise_and(img, img, mask=mask)
            gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

            ret, thresh1 = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            # cv2.imshow('gray',gray)

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (17, 3))
            closed = cv2.morphologyEx(thresh1, cv2.MORPH_CLOSE, kernel)

            _, cnts, hierarchy = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            imgRs = []
            for cnt in cnts:
                rect = cv2.minAreaRect(cnt)
                x, y, w, h = cv2.boundingRect(cnt)
                if (w > 300 or h > 200 or h > int(1.2 * w) or w > int(3.0*h) or cv2.contourArea(
                        cnt) < 200 or cv2.contourArea(cnt) > 3000):
                    continue
                cv2.drawContours(img, [cnt], -1, (0, 0, 255), 1)
                imgRs.append((x, y, w, h, rect[2]))  # rect[2] is the rotate angle

            if (len(imgRs) == 0):
                return  # no targets detected

            if self.adjust(img, imgRs) == 1:
                count = 0
                for rs in imgRs:
                    self.pos.append([rs[0], rs[1], rs[2], rs[3]])
                    # cv2.rectangle(self.returnImage,(rs[0],rs[1]),(rs[0]+rs[2],rs[1]+rs[3]), (255,0,255), 2)
                    part_image = self.returnImage[rs[1]:rs[1]+rs[3], rs[0]:rs[0]+rs[2]]
                    cv2.imwrite("rect"+str(count)+".jpg",part_image)
                    count+=1
                    cv2.imwrite("ancheck.jpg",part_image)
                    self.ret.image = cv2.imread("ancheck.jpg")
                    # return positions here
                break
            else:
                continue
                # return the target position

    # return True or False
    def ideal(self, shape, center):
        # define your method
        left = shape[1] * 1 / 3
        right = shape[1] * 2 / 3
        up = shape[0] * 1 / 3
        down = shape[0] * 2 / 3
        return (left < center[0]) & (right > center[0]) & (up < center[1]) & (down > center[1])

    # This return a array [[0,0,0],[0,0]] -> The updated region position and updated camera angle
    def translate(self, image, ret, region, camera):
        movement = [self.moveUp/6.0, 0.0, self.moveRight/6.0]
        for i in range(3):
            movement[i]+=region.center[i]
            movement[i]=float('%.2f' % movement[i])
        return [movement, camera.angle+[0.0,0.0]]
        # if moveUp is positive, then UAV should
        # I think the camera angle should be fixed and I skip this part of function

    def adjust(self, postimage, imgRs):
        # define your method
        shape = postimage.shape
        centerPoints = []
        flag = 0
        for rs in imgRs:
            x = rs[0]+rs[2]/2
            y = rs[1]+rs[3]/2
            # in case the region is too small, process in detail later
            if (rs[0]+rs[2] >= shape[1]-2) & (self.rightWidth <= 19):
                self.rightWidth += 1  # move right
                self.moveRight += 1
                flag = 1
            elif (rs[0] <= 2) & (self.leftWidth >= 1):
                self.leftWidth -= 1   # move left
                self.moveRight -= 1
                flag = 1
            elif (rs[1] <= 2) & (self.upHeight >= 1):
                self.upHeight -= 1    # move up
                self.moveUp += 1
                flag = 1
            elif (rs[1]+rs[3] >= shape[0]-2) & (self.lowHeight <= 19):
                self.lowHeight += 1   # move down
                self.moveUp -= 1
                flag = 1

            if flag == 1: return 0
            centerPoints.append((x,y))

        # prevent endless loop
        if [self.upHeight, self.lowHeight, self.leftWidth, self.rightWidth] in self.record:
            return 1
        else:
            self.record.append([self.upHeight, self.lowHeight, self.leftWidth, self.rightWidth])

        centerx = 0
        centery = 0

        for point in centerPoints:
            centerx += point[0]
            centery += point[1]
        centerx //= len(imgRs)
        centery //= len(imgRs)
        center = (centerx, centery)
        if self.ideal(shape, center):
            return 1
        else:
            # do some adjustment
            left = shape[1] * 1 / 3
            right = shape[1] * 2 / 3
            up = shape[0] * 1 / 3
            down = shape[0] * 2 / 3
            if (centerx > right): # move right
                self.leftWidth += 1
                self.moveRight += 1
            if (centerx < left): # move left
                self.rightWidth -= 1
                self.moveRight -= 1
            if (centery < up): # move up
                self.upHeight -= 1
                self.moveUp += 1
            if (centery > down): #move down
                self.lowHeight += 1
                self.moveUp -= 1

            if (self.leftWidth < 1 or self.rightWidth > 19 or self.upHeight < 1 or self.lowHeight > 19):
                return 1
            else:
                return 0

    def multi_detect(self, image):
        self.pos, threads = [], []
        [threads.append(threading.Thread(target=self.position, args=([image]))) for i in range(NUM_THREADS)]
        for t in threads:
            t.setDaemon(True)
            t.start()
        for t in threads:
            t.join()
        if len(self.pos) > 0: self.ret.exist = True

    image = None
    def postprocess(self, image, UAV):
        tar_pos = UAV.region.center[:]
        tar_pos[1]+=0.5
        UAV.move(tar_pos, self.camera.angle)
        self.image = UAV.comm.waitImage(COMPRESS)
        return