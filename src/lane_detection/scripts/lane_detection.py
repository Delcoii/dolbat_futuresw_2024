#!/usr/bin/env python3
import os

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class LaneDetection:
    def __init__(self):
        self.loop_rate = rospy.Rate(30)
        self.bridge = CvBridge()
        self.p_throttle = rospy.Publisher('/car/throttle', Float32, queue_size=10)
        self.p_steer = rospy.Publisher('/car/steering', Float32, queue_size=10)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback_image)

        self.i_image = None
        self.o_throttle = Float32()
        self.o_steer = Float32()

        # variables
        # self.prev_left_base = None
        self.prev_right_base = None
        self.fixed_points = [(6, 472), (590, 472), (396, 250), (147, 250)]

        # 픽셀 당 거리(m/pixel)
        self.x_m_per_pixel = 1.62367454495 * 10 ** -3  # 픽셀 당 거리 -> 수정됨
        self.y_m_per_pixel = 3.643428499552204 * 10 ** -3  # 픽셀 당 거리 -> 수정됨
        self.real_shift_distance = 0.45  # 실제 이동해야 할 거리 (0.45m, 즉 450mm)
        self.pixel_shift = self.real_shift_distance / self.x_m_per_pixel  # 실제 평행이동할 픽셀 거리

        self.L_m = 55 * 10 ** -2

    def callback_image(self, data):
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            self.i_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # 이미지를 윈도우에 표시
        # cv2.imshow("RGB image Image", self.i_image)
        # cv2.waitKey(1)

    def get_bird_eye_view(self, image, output_size, points):
        if len(points) != 4:
            return image

        height, width = output_size[1], output_size[0]
        src_points = np.float32([points[0], points[1], points[3], points[2]])
        dst_points = np.float32([[0, height], [width, height], [0, 0], [width, 0]])

        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        bird_eye_view = cv2.warpPerspective(image, matrix, output_size)

        return bird_eye_view

    def mask_hls_areas(self, image, lower_hls, upper_hls, blur_ksize=(15, 15)):
        # 블러링 처리
        blurred_image = cv2.GaussianBlur(image, blur_ksize, 0)
        # HLS 색 공간으로 변환
        hls_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HLS)
        # 범위에 해당하는 마스크 생성
        mask = cv2.inRange(hls_image, np.array(lower_hls), np.array(upper_hls))
        return mask

    def sliding_window_demo(self, image, nwindows=18, margin=50, minpix=1):
        global prev_left_base, prev_right_base

        lower_hls = [0, 220, 0]
        upper_hls = [180, 255, 255]

        masked_image = self.mask_hls_areas(image, lower_hls, upper_hls)
        bottom_third = masked_image.shape[0] * 2 // 3
        histogram = np.sum(masked_image[bottom_third:, :], axis=0)

        midpoint = int(histogram.shape[0] // 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        left_lane_boundary = image.shape[1] // 3
        right_lane_boundary = 1.6 * image.shape[1] // 3

        apply_left_window = left_lane_boundary >= leftx_base > 0
        apply_right_window = rightx_base >= right_lane_boundary and rightx_base > midpoint

        if not apply_left_window and prev_left_base is not None:
            leftx_base = prev_left_base
        else:
            prev_left_base = leftx_base

        if not apply_right_window and prev_right_base is not None:
            rightx_base = prev_right_base
        else:
            prev_right_base = rightx_base

        window_height = int(masked_image.shape[0] // nwindows)
        leftx_current = leftx_base
        rightx_current = rightx_base

        nonzero = masked_image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        left_lane_points_x = []
        left_lane_points_y = []
        right_lane_points_x = []
        right_lane_points_y = []

        lower_bound = image.shape[0] * 2 // 3  # 하단 1/3의 시작점 (320)
        upper_bound = image.shape[0]  # 이미지의 끝 (480)

        for window in range(nwindows):
            win_y_low = masked_image.shape[0] - (window + 1) * window_height
            win_y_high = masked_image.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            # 하단 1/3 구간만을 사용하여 왼쪽 차선 피팅
            if win_y_high >= lower_bound and win_y_low <= upper_bound and apply_left_window:
                good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                  (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
                if len(good_left_inds) > minpix:
                    leftx_current = int(np.mean(nonzerox[good_left_inds]))
                    left_lane_points_x.append(leftx_current)
                    left_lane_points_y.append(int(np.mean(nonzeroy[good_left_inds])))

            # 전체 구간에서 오른쪽 차선 피팅
            if apply_right_window:
                good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                   (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
                if len(good_right_inds) > minpix:
                    rightx_current = int(np.mean(nonzerox[good_right_inds]))
                    right_lane_points_x.append(rightx_current)
                    right_lane_points_y.append(int(np.mean(nonzeroy[good_right_inds])))

        # 왼쪽 차선에 대한 1차 다항식 구하기 (하단 1/3 구간에서만)
        if len(left_lane_points_x) >= 2 and len(left_lane_points_y) >= 2:
            left_fit = np.polyfit(left_lane_points_y, left_lane_points_x, 1)  # 1차 함수로 피팅
        else:
            left_fit = None

        # 오른쪽 차선에 대한 3차 다항식 구하기 (하단 1/2 구간에서 데이터 포인트가 5개 이상일 때만)
        if len(right_lane_points_x) >= 4 and len(right_lane_points_y) >= 4 and len(
                [y for y in right_lane_points_y if y >= lower_bound]) >= 5:
            right_fit = np.polyfit(right_lane_points_y, right_lane_points_x, 3)
        else:
            right_fit = None

        return left_fit, right_fit

    def calculate_path(self, left_fit, right_fit, image_shape, pixel_shift):
        ploty = np.linspace(0, image_shape[0] - 1, image_shape[0])

        if right_fit is not None:
            # 오른쪽 차선만 감지된 경우
            right_fitx = right_fit[0] * ploty ** 3 + right_fit[1] * ploty ** 2 + right_fit[2] * ploty + right_fit[3]
            center_fitx = right_fitx - pixel_shift
            return ploty, center_fitx, None, right_fitx

        elif left_fit is not None:
            # 왼쪽 차선만 감지된 경우
            left_fitx = left_fit[0] * ploty + left_fit[1]  # 1차 함수로 계산
            center_fitx = left_fitx + pixel_shift
            return ploty, center_fitx, left_fitx, None

        else:
            return None, None, None, None

    steering_angles = []  # 조향각 히스토리 저장
    window_size = 5  # 이동 평균 윈도우 크기

    def moving_average_filter(self, new_angle):
        self.steering_angles.append(new_angle)
        if len(self.steering_angles) > self.window_size:
            self.steering_angles.pop(0)
        return np.mean(self.steering_angles)

    def calculate_steering_angle(self, x_la, y_la):
        # Pure Pursuit을 사용한 조향각 계산
        steering_angle = np.arctan2(2 * self.L_m * y_la, x_la ** 2 + y_la ** 2)
        steering_angle_deg = self.moving_average_filter(np.degrees(steering_angle))

        return steering_angle_deg

    def calculate_steering_angle_from_slope(self, slope):
        # 차선의 기울기(슬로프)를 사용하여 조향각을 계산
        steering_angle_deg = self.moving_average_filter(np.degrees(np.arctan(slope)))

        # 조향각이 -25도 이하로 크면 -16도로 제한
        if steering_angle_deg < -25:
            return -16
        return steering_angle_deg

    def update_output(self, throttle, steer):
        self.o_throttle = throttle
        self.o_steer = steer

    def publish(self):
        self.p_throttle.publish(self.o_throttle)
        self.p_steer.publish(self.o_steer)

    def run(self):

        # plt.ion()  # 인터랙티브 모드 켜기
        # fig, ax = plt.subplots()

        # 축의 범위 설정

        x_la = 1.5  # Lookahead X value in meters

        while not rospy.is_shutdown():
            frame = self.i_image

            if frame is None:
                print("no image read")
                self.loop_rate.sleep()
                continue

            bev = self.get_bird_eye_view(frame, (frame.shape[1], frame.shape[0]), self.fixed_points)
            left_fit, right_fit = self.sliding_window_demo(bev)
            ploty, center_fitx, left_fitx, right_fitx = self.calculate_path(left_fit, right_fit, bev.shape,
                                                                            self.pixel_shift)

            if ploty is not None and center_fitx is not None:

                # Convert coordinates to meters
                ploty_m = (480 - ploty) * self.y_m_per_pixel

                # if left_fitx is not None:
                #     left_fitx_m = (340 - left_fitx) * self.x_m_per_pixel

                if right_fitx is not None:
                    right_fitx_m = (340 - right_fitx) * self.x_m_per_pixel

                center_fitx_m = (340 - center_fitx) * self.x_m_per_pixel

                # Calculate y_la corresponding to x_la
                center_fit_coeffs = np.polyfit(ploty_m, center_fitx_m, 3)
                y_la = np.polyval(center_fit_coeffs, x_la)

                # Calculate steering angle
                steering_angle = self.calculate_steering_angle(x_la, y_la)
                print(f"Steering Angle: {steering_angle:.2f} degrees")

            throttle = 0.2  # for quiet sound
            self.update_output(throttle, steering_angle)
            self.publish()

            cv2.imshow("Video", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            self.loop_rate.sleep()


def main():
    rospy.init_node('lane_detection', anonymous=True)
    node = LaneDetection()
    node.run()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
