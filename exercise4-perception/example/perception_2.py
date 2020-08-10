# coding:utf-8

import cv2
import numpy as np

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


def abs_sobel_thresh(new_image, sobel_kernel=3, orient='x', thresh=(20, 255)):
    # Convert Color from BGR2GRAY
    gray = cv2.cvtColor(new_image, cv2.COLOR_BGR2GRAY)

    # sobel
    if orient == 'x':
        sobel = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    else:
        sobel = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)

    abs_sobel = np.absolute(sobel)
    scaled_sobel = np.uint8(255 * abs_sobel / np.max(abs_sobel))
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1
    return sxbinary


if __name__ == '__main__':
    src = cv2.imread('./data/4-11.png')
    src = cv2.resize(src, (640, 360))
    img = perspective_transform(src, M, (580, 560))

    while True:
        cv2.imshow('input_image', img)

        # convert color from BGR to HSV
        image_sobel = abs_sobel_thresh(img, orient='x')

        temp_2 = np.dstack((image_sobel, image_sobel, image_sobel)) * 255

        cv2.imshow('image_sobel', temp_2)

        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


