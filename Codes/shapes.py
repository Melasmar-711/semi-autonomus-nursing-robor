#!/usr/bin/env python 

import cv2
import numpy as np 


if __name__ == '__main__':
	
	#img_path = 'images/index10.png'
	
	#img = cv2.imread(img_path)
	
	#img_grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	
	cap = cv2.VideoCapture(0)
	cap.set(3,640)
	cap.set(4,480)
	
	
	while True:
	
		success,img = cap.read()
		
		img_grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		#_,thrash = cv2.threshold (img_grey , 127 , 255 , cv2.THRESH_BINARY)
		blur = cv2.GaussianBlur(img_grey,(5,5),0)
		ret3,th3 = cv2.threshold(blur,100,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

		contours,_ = cv2.findContours(th3,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
		
		for contour in contours:
	
			
			approx = cv2.approxPolyDP(contour, 0.01*cv2.arcLength(contour,True),True)
			cv2.drawContours(img,[approx],0,(0, 0, 0),5)
			x = approx.ravel()[0]
			y = approx.ravel()[1]
		
			if len (approx) == 3 :
				cv2.putText(img, 'Triangle', (x,y),cv2.FONT_HERSHEY_COMPLEX, 0.5 , (0 , 0 , 0))
				print ('Triangle')
				#break
				
			if len (approx) == 4 :
				x,y,w,h = cv2.boundingRect(approx)
				asr = float(w)/h 
				print (asr)
				if  asr >=0.95 and asr <=1.05:
					cv2.putText(img, 'Square', (x,y),cv2.FONT_HERSHEY_COMPLEX, 0.5 , (0 , 0 , 0))
					print ('Square')
				else:
					cv2.putText(img, 'Rectangle', (x,y),cv2.FONT_HERSHEY_COMPLEX, 0.5 , (0 , 0 , 0))
					print ('Rectangle')
			
			elif len (approx) == 5 :
				cv2.putText(img, 'Pentagon', (x,y),cv2.FONT_HERSHEY_COMPLEX, 0.5 , (0 , 0 , 0))
				print ('Pentagon')
			elif len (approx) == 10 :
				cv2.putText(img, 'Star', (x,y),cv2.FONT_HERSHEY_COMPLEX, 0.5 , (0 , 0 , 0))
				print ('Star')
	
			else :
				cv2.putText(img, 'Circle', (x,y),cv2.FONT_HERSHEY_COMPLEX, 0.5 , (0 , 0 , 0))
				print ('Circle')
		
		cv2.imshow ('shapes' , img)
		cv2.waitKey(1)
	
	
	
	 

