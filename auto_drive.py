import cv2
import numpy as np
from math import hypot
import datetime

INF = 9999

def printHSV(event, x, y, flags, hsv):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("({}, {})".format(y, x), 'HSV:', hsv[y, x])

def isPointInPolygon(point, rangelist):
  lnglist = []
  latlist = []
  for i in range(len(rangelist)-1):
    lnglist.append(rangelist[i][0])
    latlist.append(rangelist[i][1])
  maxlng = max(lnglist)
  minlng = min(lnglist)
  maxlat = max(latlist)
  minlat = min(latlist)
  if (point[0] > maxlng or point[0] < minlng or
    point[1] > maxlat or point[1] < minlat):
    return False
  count = 0
  point1 = rangelist[0]
  for i in range(1, len(rangelist)):
    point2 = rangelist[i]
    # 点与多边形顶点重合
    if (point[0] == point1[0] and point[1] == point1[1]) or (point[0] == point2[0] and point[1] == point2[1]):
      return False
    # 判断线段两端点是否在射线两侧 不在肯定不相交 射线（-∞，lat）（lng,lat）
    if (point1[1] < point[1] and point2[1] >= point[1]) or (point1[1] >= point[1] and point2[1] < point[1]):
      # 求线段与射线交点 再和lat比较
      point12lng = point2[0] - (point2[1] - point[1]) * (point2[0] - point1[0])/(point2[1] - point1[1])
      # 点在多边形边上
      if (point12lng == point[0]):
        return False
      if (point12lng < point[0]):
        count +=1
    point1 = point2
  if count%2 == 0:
    return False
  else:
    return True

def isInROI(roi, line):
    x1, y1, x2, y2 = line[0]
    # print("x1:{}, y1:{}, x2:{}, y2:{}".format(x1, y1, x2, y2))
    # print(roi[0])
    # print(roi[1])
    # print(roi[2])
    # print(roi[3])
    return isPointInPolygon([x1, y1], roi) and isPointInPolygon([x2, y2], roi)

def get_distance(pt1, pt2):
    return hypot(pt1[0] - pt2[0], pt1[1] - pt2[1]) 

def updateROIByNearset(roi, illegal_points, max_drift):
    cloest_top_left = INF
    cloest_bottom_left = INF
    cloest_bottom_right = INF
    cloest_top_right = INF

    for point in illegal_points:
        if get_distance(point, roi[0]) < cloest_top_left:
            top_left = point
            cloest_top_left = get_distance(point, roi[0])
        if get_distance(point, roi[1]) < cloest_bottom_left:
            bottom_left = point
            cloest_bottom_left = get_distance(point, roi[1])
        if get_distance(point, roi[2]) < cloest_bottom_right:
            bottom_right = point
            cloest_bottom_right = get_distance(point, roi[2])
        if get_distance(point, roi[3]) < cloest_top_right:
            top_right = point
            cloest_top_right = get_distance(point, roi[3])
    
     # limit the ROI drift, can't change sharply
    res_top_left = top_left if cloest_top_left < max_drift else roi[0]
    res_bottom_left = bottom_left if cloest_bottom_left < max_drift else roi[1]
    res_bottom_right = bottom_right if cloest_bottom_right < max_drift else roi[2]
    res_top_right = top_right if cloest_top_right < max_drift else roi[3] 
    
    return np.array([[res_top_left[0], 490], [res_bottom_left[0], roi[1][1]], [res_bottom_right[0], roi[2][1]], [res_top_right[0], 490]])

def updateROI(roi, illegal_points, max_drift):
    # sort the points by distance to the ROI Center as Descend
    center_point = [int((roi[1][0] + roi[2][0]) / 2), int((roi[0][1] + roi[1][1]) / 2)]
    # print(" ->Before:", illegal_points)
    illegal_points.sort(key=lambda element: get_distance(center_point, element), reverse=True)
    # print(" ->After:", illegal_points)
    corner_has_point = [0, 0, 0, 0]
    new_roi_list = [roi[0], roi[1], roi[2], roi[3]]
    close_corner_list = [INF, INF, INF, INF]

    print("Point Length:", len(illegal_points))
    for point in illegal_points:
        distance_to_each_corner = np.array([get_distance(point, roi[0]), get_distance(point, roi[1]), get_distance(point, roi[2]), get_distance(point, roi[3])])
        corner_idx = np.argmin(distance_to_each_corner)
        # Just Update those corners which don't have corresponding point
        if corner_has_point[corner_idx] == 0:
            close_corner_list[corner_idx] = distance_to_each_corner[corner_idx]
            new_roi_list[corner_idx] = point
            corner_has_point[corner_idx] = 1        
        print("Corner Has Point: ", corner_has_point)
        print("New Roi List: ", corner_has_point)
        print("Close Distance: ", close_corner_list)
        print("Point", point)
        print("------------------------------------")
        if sum(corner_has_point) == 4:
            break
    # limit the ROI drift, can't change sharply
    res_top_left = new_roi_list[0] if close_corner_list[0] < max_drift else roi[0]
    res_bottom_left = new_roi_list[1] if close_corner_list[1] < max_drift else roi[1]
    res_bottom_right = new_roi_list[2] if close_corner_list[2] < max_drift else roi[2]
    res_top_right = new_roi_list[3] if close_corner_list[3] < max_drift else roi[3] 
    
    return np.array([[res_top_left[0], 490], [res_bottom_left[0], roi[1][1]], [res_bottom_right[0], roi[2][1]], [res_top_right[0], 490]])

def notInButCloseToROI(roi, pt, close_threshold):
    if get_distance(roi[0], pt) < close_threshold or get_distance(roi[1], pt) < close_threshold or get_distance(roi[2], pt) < close_threshold or get_distance(roi[3], pt) < close_threshold:
        return True
    return False

def drawROI(frame, roi):
    roi_mask = np.zeros_like(frame)
    cv2.fillPoly(roi_mask, [roi], (255, 255, 0))
    return cv2.addWeighted(frame, 1, roi_mask, 0.2, 1)


def make_coordinates(frame, line_parameters):
    slope, intercept = line_parameters
    y1 = int(frame.shape[0])
    y2 = int(y1 * 2 / 3)
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])

def drawLaneLine(frame, lines):
    left_fit = []
    right_fit = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        intercept = parameters[1]
        if slope < 0:
            left_fit.append([slope, intercept])
        else:
            right_fit.append([slope, intercept])
    
    
    lane_line_mask = np.zeros_like(frame)
    if len(left_fit):
        # print("Left", left_fit)
        left_fit_average = np.average(left_fit, axis=0)
        left_line = make_coordinates(frame, left_fit_average)
        cv2.line(lane_line_mask, (left_line[0], left_line[1]), (left_line[2], left_line[3]), (0, 0, 255), 3)
    if len(right_fit):
        # print("Right", right_fit) 
        right_fit_average = np.average(right_fit, axis=0)   
        right_line = make_coordinates(frame, right_fit_average)
        cv2.line(lane_line_mask, (right_line[0], right_line[1]), (right_line[2], right_line[3]), (0, 0, 255), 3)
    return cv2.addWeighted(frame, 1, lane_line_mask, 1, 1)


def main():
    video = cv2.VideoCapture("road_car_view2.mp4")
    width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cv2.namedWindow('Auto Drive')
    cv2.createTrackbar('Max Drift', 'Auto Drive', 60, 200, lambda x: None)
    cv2.createTrackbar('Close', 'Auto Drive', 150, 300, lambda x: None)
    show_flag = 'RGB'

    print("->Frame Property: ", width, height)

    # (x, y) -> [left_top, left_bottom, right_bottom, right_top]
    roi = np.array([[567, 490], [133, height], [1109, height], [786, 490]])

    while True:
        ret, orig_frame = video.read()
        if not ret:
            video = cv2.VideoCapture("road_car_view2.mp4")
            roi = np.array([[567, 490], [133, height], [1109, height], [786, 490]])
            continue

        frame = cv2.GaussianBlur(orig_frame, (5, 5), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cv2.setMouseCallback('Auto Drive', printHSV, hsv)
        
        low_yellow = np.array([18, 94, 140])
        up_yellow = np.array([48, 255, 255])
        mask = cv2.inRange(hsv, low_yellow, up_yellow)
        edges = cv2.Canny(mask, 75, 150)
        frame_edges = cv2.Canny(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 75, 150)

        if show_flag == 'RGB':
            # cv2.polylines(orig_frame, [roi], 1, (255, 255, 0), 2)
            orig_frame = drawROI(orig_frame, roi)
            cv2.putText(orig_frame, "Press 'E' - Edges View", (10, 55), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 255, 0))
            cv2.putText(orig_frame, "Press 'R' - RGB View", (10, 30), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 255, 0))
            cv2.putText(orig_frame, datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'), (int(width * 0.81), 30), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 255, 0))
        elif show_flag == 'EDGE':
            cv2.polylines(frame_edges, [roi], 1, (255, 0, 0), 2)

        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 80, maxLineGap=250)
        illegal_points = []
        illegal_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.circle(orig_frame, (x1, y1), 2, (0, 255, 0), -1)
                cv2.circle(orig_frame, (x2, y2), 2, (0, 255, 0), -1)
                if isInROI(roi, line):
                    x1, y1, x2, y2 = line[0]
                    # cv2.line(orig_frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
                    illegal_points.append([x1, y1])
                    illegal_points.append([x2, y2])
                    illegal_lines.append(line)
                    # print('({}, {}), ({}, {})'.format(x1, y1, x2, y2))
                else:
                    x1, y1, x2, y2 = line[0]
                    thres = cv2.getTrackbarPos('Close', 'Auto Drive')
                    if notInButCloseToROI(roi, [x1, y1], close_threshold=thres):
                        illegal_points.append([x1, y1])
                    if notInButCloseToROI(roi, [x2, y2], close_threshold=thres):
                        illegal_points.append([x2, y2])

        # update ROI
        drift = cv2.getTrackbarPos('Max Drift', 'Auto Drive')
        roi = updateROI(roi, illegal_points, max_drift=drift)

        # Draw Lane Line
        if illegal_lines is not None:
            orig_frame = drawLaneLine(orig_frame, illegal_lines)

        if show_flag == 'RGB':
            cv2.imshow("Auto Drive", orig_frame)
        elif show_flag == 'EDGE':
            cv2.imshow("Auto Drive", frame_edges)
        # cv2.imshow("edges", edges)
        # print('--------------------------------------')
        key = cv2.waitKey(33)
        if key == ord('q'):
            break
        elif key == ord('e'):
            show_flag = 'EDGE'
        elif key == ord('r'):
            show_flag = 'RGB'
    video.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()