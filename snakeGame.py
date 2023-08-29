import math
import random

import cvzone
import cv2
import numpy as np
from cvzone.HandTrackingModule import HandDetector

cap= cv2.VideoCapture(0)
cap.set(3, 1300)
cap.set(4, 750)

detector= HandDetector(detectionCon=0.8,maxHands=1)

#for the game we need List of points, List of Distances, Current Length, Total Length
class SnakeGameClass:
    def __init__(self,pathFood):
        self.points= [] # list of all points of the snake
        self.length= [] # distance b/w each point
        self.currentLength= 0 # total length of th snake
        self.allowedLength= 150 # total allowed length and whenever snake eat the food, it started increasing 150+
        self.previousHead= 0, 0 # previous head point

        self.imgFood= cv2.imread(pathFood, cv2.IMREAD_UNCHANGED)
        self.heightFood, self.widthFood,_ = self.imgFood.shape
        self.foodPoints= 0, 0
        self.randomFoodLocation()
        self.score = 0
        self.gameOver= False

    def randomFoodLocation(self):
        self.foodPoints= random.randint(100,1000), random.randint(100,600)

    # function for updating
    def update(self, imgMain, currentHead):

        if self.gameOver:
            cvzone.putTextRect(imgMain,"Game Over",[300,400],scale=7,thickness=5,offset=20)
            cvzone.putTextRect(imgMain,f"Your Score:{self.score}",[300,550],scale=7,thickness=5,offset=20)

        else:

            px, py = self.previousHead
            cx, cy = currentHead
            # now find distance b/w current head and previous head
            self.points.append([cx, cy])
            distance= math.hypot(cx - px, cy - py) # with help of hypoytenuse method we can easily find distance in one single line
            self.length.append(distance)
            self.currentLength += distance
            self.previousHead=cx, cy # for next iteration

            # Length Reduction
            if self.currentLength > self.allowedLength:
                for i, lgth in enumerate(self.length):
                    self.currentLength -= lgth
                    self.length.pop(i)
                    self.points.pop(i)

                    if self.currentLength < self.allowedLength:
                        break

            # Check if snake ate the food or not
            rx, ry = self.foodPoints
            if (rx -self.widthFood//2 <cx< rx +self.widthFood//2 and
                    ry -self.heightFood <cy< ry +self.heightFood):
                # print("ate" )
                self.randomFoodLocation()
                self.allowedLength +=50
                self.score += 1
                print(self.score)




            # Draw Snake
            if self.points:
                for i, point in enumerate(self.points): # enumerate will giev the iteration number
                    if i!= 0:
                        cv2.line(imgMain,self.points[i-1],self.points[i],(0,0,255),18)
                cv2.circle(imgMain, self.points[-1], 20, (200, 0, 200), cv2.FILLED)

            # Draw Food
            imgMain= cvzone.overlayPNG(imgMain,self.imgFood,(rx-self.widthFood// 2,ry-self.heightFood// 2))
            cvzone.putTextRect(imgMain, f"Score:{self.score}", [50, 80], scale=3, thickness=3, offset=10)

            # Check for Collision
            pts = np.array(self.points[:-2], np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(imgMain, [pts], False, (0, 200, 0), 3)
            minDist= cv2.pointPolygonTest(pts,(cx, cy),True)
            # print(minDist)

            if -1<= minDist <=1:
                print("Hit")
                self.gameOver= True
                self.points = []  # list of all points of the snake
                self.length = []  # distance b/w each point
                self.currentLength = 0  # total length of th snake
                self.allowedLength = 150  # total allowed length and whenever snake eat the food, it started increasing 150+
                self.previousHead = 0, 0  # previous head point

                self.randomFoodLocation()

        return imgMain

game=SnakeGameClass("Donut.png")

while True:
    success, img= cap.read()
    # for changing the orientation of the hand, 1 is for horizontal and 0 is for vertical and fliptype fasle will help
    # user to easily understand which hand is working or not
    img= cv2.flip(img,1)
    hands,img= detector.findHands(img,flipType=False)

    # finding the index of hands finger to detect which finger is moving the snake
    if hands:
        lmlist= hands[0]['lmList']
        pointIndex= lmlist[8][0:2]
        img= game.update(img,pointIndex)
    cv2.imshow("Image",img)
    key= cv2.waitKey(1)
    if key== ord('r'):
        game.gameOver= False