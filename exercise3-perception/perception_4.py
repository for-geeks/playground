# coding:utf-8

import cv2
import numpy as np
from preception_1 import color_mask
from preception_2 import abs_sobel_thresh
from preception_3 import merge_mask

# roll
src_corners = [[274, 250], [438, 252], [250, 339], [502, 341]]
# turn to
dst_corners = [[262, 470], [342, 470], [262, 550], [342, 550]]

M = cv2.getPerspectiveTransform(np.float32(src_corners), np.float32(dst_corners))


def perspective_transform(image, m, img_size=None):
    if img_size is None:
        img_size = (image.shape[1], image.shape[0])
    warped = cv2.warpPerspective(image, m, img_size, flags=cv2.INTER_LINEAR)
    return warped


def get_win_point(leftx, lefty, shape):

    left_x_re = []

    tag_y = shape[0] - 1

    get_polt_tag = 0

    bs_tag = int(shape[1] / 50)

    pox_arr_x_l = []
    pox_arr_y = []

    while tag_y >= 0:
        left_x_0 = -1

        if tag_y in lefty:
            for i_y, ic in enumerate(lefty):
                if ic == tag_y:
                    left_x_0 = leftx[i_y]
                    break

        if left_x_0 == -1:
            tag_y -= 1
            get_polt_tag += 1
            continue

        if get_polt_tag < bs_tag:
            pox_arr_x_l.append(left_x_0)

            pox_arr_y.append(tag_y)
        else:
            if len(pox_arr_y) > 0:
                x_l = int(np.sum(pox_arr_x_l) // len(pox_arr_x_l))
                left_x_re.append(x_l)

                pox_arr_x_l = []
                pox_arr_y = []

            get_polt_tag = 0

        tag_y -= 1
        get_polt_tag += 1

    return left_x_re


def find_line_fit(image_input, midpoint=None, nwindows=20, margin=100, minpix=30):
    histogram = np.sum(image_input[img.shape[0] // 2:, :], axis=0)
    leftx_base = np.argmax(histogram)

    # Set height of windows
    window_height = np.int(image_input.shape[0] / nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = image_input.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base
    line_rank_x = []
    line_rank_y = []

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = img.shape[0] - (window + 1) * window_height
        win_y_high = img.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin

        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        # Append these indices to the lists

        left_lane_inds.append(good_left_inds)

        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            line_rank_x.append(leftx_current)
            line_rank_y.append((win_y_low+win_y_high)//2)

    left_list = get_win_point(line_rank_x, line_rank_y, img.shape)

    return left_list, line_rank_x, line_rank_y


if __name__ == '__main__':
    src = cv2.imread('./data/4-11.png')
    src = cv2.resize(src, (640, 360))
    img = perspective_transform(src, M, (580, 560))
    car_mid_point = 228

    while True:
        img_res = np.copy(img)

        image_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_y = color_mask(image_HSV)
        mask_s = abs_sobel_thresh(img, orient='x')
        merge_img = merge_mask(mask_y, mask_s)

        temp_2 = np.dstack((merge_img, merge_img, merge_img)) * 255

        # get line
        line_list, mean_x, mean_y = find_line_fit(
            merge_img, midpoint=car_mid_point, nwindows=10, margin=100)

        # size:
        col_weight_half = 100/2
        col_height_half = 560/20

        for i, x_a in enumerate(mean_x):
            cv2.rectangle(img_res, (int(x_a - col_weight_half), int(mean_y[i]-col_height_half)),
                          (int(x_a + col_weight_half), int(mean_y[i]+col_height_half)),
                          (255, 255, 255), thickness=2)
            cv2.rectangle(temp_2, (int(x_a - col_weight_half), int(mean_y[i] - col_height_half)),
                          (int(x_a + col_weight_half), int(mean_y[i] + col_height_half)),
                          (0, 0, 255), thickness=2)

        cv2.imshow('image_sobel', temp_2)

        cv2.imshow('image', img_res)

        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


