import cv2
import numpy as np
from pyzbar.pyzbar import decode

if __name__ == '__main__':

	#img_path = 'images/image_formation_example.png'
	#img = cv2.imread (img_path)
	
	cap = cv2.VideoCapture(0)
	cap.set(3,640)
	cap.set(4,480)
	
	while True:
		
		success,img = cap.read()
		
		for barcode in decode (img):
	
			print (barcode.data)
			mydata = barcode.data.decode('utf-8')
			print (mydata)
		
		cv2.imshow('result',img)
		cv2.waitKey(1) 
