# coding:utf-8

import cv2
import numpy as np
from preception_1 import color_mask
from preception_2 import abs_sobel_thresh

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


def merge_mask(mask_yellow, mask_sobel):

    # Fix here
    mask_yellow[(mask_yellow != 0)] = 1
    combined = np.zeros_like(mask_sobel)
    combined[((mask_sobel == 1) & (mask_yellow == 1))] = 1

    return combined


if __name__ == '__main__':
    src = cv2.imread('./data/4-11.png')
    src = cv2.resize(src, (640, 360))
    img = perspective_transform(src, M, (580, 560))

    while True:
        cv2.imshow('input_image', img)

        image_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_y = color_mask(image_HSV)

        cv2.imshow('image_y', mask_y)

        mask_s = abs_sobel_thresh(img, orient='x')

        merge_img = merge_mask(mask_y, mask_s)

        cv2.imshow('image_s', mask_s*255)

        temp_2 = np.dstack((merge_img, merge_img, merge_img)) * 255

        cv2.imshow('image_sobel', temp_2)

        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


