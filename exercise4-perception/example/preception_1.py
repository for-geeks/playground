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


def color_mask(image_HSV):
    # Return mask from HSV
    yellow_hsv_low = np.array([10, 100, 20])
    yellow_hsv_high = np.array([25, 255, 255])
    mask = cv2.inRange(image_HSV, yellow_hsv_low, yellow_hsv_high)
    return mask


if __name__ == '__main__':
    src = cv2.imread('./data/4-11.png')
    src = cv2.resize(src, (640, 360))
    img = perspective_transform(src, M, (580, 560))

    while True:
        cv2.imshow('input_image', img)

        # convert color from BGR to HSV
        image_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mark_yellow = color_mask(image_HSV)

        cv2.imshow('mark_yellow', mark_yellow)

        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


