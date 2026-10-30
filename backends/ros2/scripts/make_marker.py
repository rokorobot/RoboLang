"""Write an ArUco marker (DICT_4X4_50, id 7) with a white border to the PNG path given as the only argument."""
import sys

import cv2

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker = cv2.aruco.generateImageMarker(dictionary, 7, 400)
cv2.imwrite(sys.argv[1], cv2.copyMakeBorder(marker, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255))
