import cv2
import cv2.aruco as aruco

# dictionary 선택 (4x4_50, 5x5_100, 6x6_250, 7x7_1000 등)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# 마커 생성: id, size(px)
marker_id = 23
marker_size = 700  # 픽셀
marker_img = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

cv2.imwrite(f"marker_{marker_id}.png", marker_img)