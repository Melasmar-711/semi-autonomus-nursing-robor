import cv2
import mediapipe as mp
import time
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class PoseDetector:

    def __init__(self, mode=False, upBody=False, smooth=True, detectionCon=0.5, trackCon=0.5):

        self.mode = mode
        self.upBody = upBody
        self.smooth = smooth
        self.enableSegmentation = False
        self.smoothSegmentation = True
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpDraw = mp.solutions.drawing_utils
        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(self.mode, self.upBody, self.smooth,self.enableSegmentation, self.smoothSegmentation, self.detectionCon, self.trackCon)

    def findPose(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)
        #print(self.results.pose_landmarks)
        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(img, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)
        return img

    def getPosition(self, img, draw=False):
        lmList = []
        if self.results.pose_landmarks:
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                h, w, c = img.shape
                # print(id, lm)
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 5, (255, 0, 0), cv2.FILLED)
        return lmList


if __name__ == "__main__":
    rospy.init_node("videoExtracting_node")
    detector = PoseDetector()

    while True:
        pTime = 0

        cap = rospy.wait_for_message("sim_ros_interface/image", Image)
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(cap, desired_encoding='passthrough')
        img = cv2.flip(img, 0)

        img = detector.findPose(img)
        lmList = detector.getPosition(img)

        if abs(lmList[0][2] - lmList[29][2]) < 110:
            print("Falling")

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime

        cv2.putText(img, str(int(fps)), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
        cv2.imshow("Image", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
