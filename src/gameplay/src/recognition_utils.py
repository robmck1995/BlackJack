#! /usr/bin/python

# Code for this file was taken from:
# https://github.com/arnabdotorg/Playing-Card-Recognition
# And then modified for our purposes

import sys
import numpy as np
import cv2

# global variable to contain training on whole deck
training = None


###############################################################################
# Utility code from 
# http://git.io/vGi60A
# Thanks to author of the sudoku example for the wonderful blog posts!
###############################################################################

def rectify(h):
  h = h.reshape((4,2))
  hnew = np.zeros((4,2),dtype = np.float32)

  add = h.sum(1)
  hnew[0] = h[np.argmin(add)]
  hnew[2] = h[np.argmax(add)]
   
  diff = np.diff(h,axis = 1)
  hnew[1] = h[np.argmin(diff)]
  hnew[3] = h[np.argmax(diff)]

  return hnew

###############################################################################
# Image Matching
###############################################################################
def preprocess(img):
  gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
  blur = cv2.GaussianBlur(gray,(5,5),2 )
  thresh = cv2.adaptiveThreshold(blur,255,1,1,11,1)
  return thresh
  
def imgdiff(img1,img2):
  img1 = cv2.GaussianBlur(img1,(5,5),5)
  img2 = cv2.GaussianBlur(img2,(5,5),5)    
  diff = cv2.absdiff(img1,img2)  
  diff = cv2.GaussianBlur(diff,(5,5),5)    
  flag, diff = cv2.threshold(diff, 200, 255, cv2.THRESH_BINARY) 
  return np.sum(diff)  

def find_closest_card(training,img):
  features = preprocess(img)
  return sorted(training.values(), key=lambda x:imgdiff(x[1],features))[0][0]
  
   
###############################################################################
# Card Extraction
###############################################################################  
def getCards(im, numcards=1):
  gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
  blur = cv2.GaussianBlur(gray,(1,1),1000)
  flag, thresh = cv2.threshold(blur, 120, 255, cv2.THRESH_BINARY) 
       
  contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

  contours = sorted(contours, key=cv2.contourArea,reverse=True)[:numcards]  

  for card in contours:
    peri = cv2.arcLength(card,True)
    approx = rectify(cv2.approxPolyDP(card,0.02*peri,True))  
    
    h = np.array([ [0,0],[449,0],[449,449],[0,449] ],np.float32)

    transform = cv2.getPerspectiveTransform(approx,h)
    warp = cv2.warpPerspective(im,transform,(450,450))
    
    yield warp


def get_training(training_labels_filename,training_image_filename,num_training_cards,avoid_cards=None):
  training = {}
  
  labels = {}
  for line in file(training_labels_filename): 
    key, num, suit = line.strip().split()
    labels[int(key)] = (num,suit)
    
  print "Training"

  im = cv2.imread(training_image_filename)
  for i,c in enumerate(getCards(im,num_training_cards)):
    if avoid_cards is None or (labels[i][0] not in avoid_cards[0] and labels[i][1] not in avoid_cards[1]):
      training[i] = (labels[i], preprocess(c))
  
  print "Done training"
  return training


# runs training on whole deck, only need to do once at beginning
def training():
  global training

  training_image_filename = "/home/cc/ee106a/fa17/class/ee106a-abe/ros_workspaces/BlackJack/src/card_recognition/src/train.png"
  training_labels_filename = "/home/cc/ee106a/fa17/class/ee106a-abe/ros_workspaces/BlackJack/src/card_recognition/src/train.tsv"
  num_training_cards = 56

  training = get_training(training_labels_filename, training_image_filename, num_training_cards)


# compares current card to training to find best match
def recognize_card(filename):
  global training
  num_cards = 1

  im = cv2.imread(filename)

  # rotates image before diff calc
  width = im.shape[0]
  height = im.shape[1]
  if width < height:
    im = cv2.transpose(im)
    im = cv2.flip(im,1)

  cards = [find_closest_card(training,c) for c in getCards(im,num_cards)]

  return cards