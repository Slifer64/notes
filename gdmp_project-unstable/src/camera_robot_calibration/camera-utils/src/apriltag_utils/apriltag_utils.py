#   Aristotle Univercisty of Thessaloniki
#   Robotics and Automation Lab
#
#   Author: Savvas Sampaziotis
#   e-mail: savvas.sampaziotis@gmail.com
#
import cv2
import numpy as np

#
#   draws point and write "P+<index>" on april tag
#
def draw_aprilcorner(np_image, x, y, index):
    alphabet_ = 'ABCD'
    np_image = cv2.circle(np_image, (int(x), int(y)), radius=10, color=(0, 255, 255), thickness=5)
    if not index == None:
        np_image = cv2.putText(np_image, alphabet_[index], (int(x), int(y)), cv2.FONT_HERSHEY_PLAIN, 5, (0, 0, 255), 5)
    # np_image = cv2.putText(np_image, "P=" + str(index), (int(x), int(y)), cv2.FONT_HERSHEY_PLAIN, 5, (0, 0, 255), 5)
    return np_image


def draw_apriltag(R, np_image):  # R for "April Tag Result"
    corners_ = np.round(R.corners)
    for i in range(0, 4):
        np_image = draw_aprilcorner(np_image, corners_[i, 0], corners_[i, 1], i )

    # write Tag-id at the center of the tag
    np_image = cv2.putText(np_image, "ID=" + str(R.tag_id), (int(R.center[0]), int(R.center[1])),
                           cv2.FONT_HERSHEY_PLAIN, 5, (0, 0, 255), 5)
    return np_image


def print_points(title_str, point_list):
    print("##" + title_str)
    for p in point_list:
        print(np.transpose(p))
    print("##" )




def draw_axis(img, R, t, camera_params, TAG_SIZE):
    fx,fy,cx,cy = camera_params[0], camera_params[1], camera_params[2], camera_params[3]
    K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
    K = np.array(K, dtype='float')

    # unit is mm
    rotV = cv2.Rodrigues(R)

    S = TAG_SIZE/2
    points = np.float32( [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, 0]]).reshape(-1, 3)
    points = S*points
    axisPoints = cv2.projectPoints(points, np.array(rotV[0]), t.T, K, (0, 0, 0, 0))
    axisPoints = axisPoints[0]
    axisPoints = axisPoints.astype(int)
    img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[0].ravel()), (255, 0, 0), 3)
    img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[1].ravel()), (0, 255, 0), 3)
    img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[2].ravel()), (0,0,255), 3)
    return img
