#
# Written by Roy Zohar, 2017.
# Published under the MIT license.
#

import cv2 as cv
import numpy as np
import math

import logging

# Image processing techniques
# relies heavily on OpenCV
class ImageProcessing(object):
    THRESHOLD = 30


    @staticmethod
    def binarizeImageInv(image):
        logging.Debug.debug("Entered binarizeImageInv")

        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        logging.Debug.debug("grayscale")

        ret, image = cv.threshold(
            image,
            np.mean(image) - ImageProcessing.THRESHOLD,
            255,
            cv.THRESH_BINARY_INV
        )

        image = cv.convertScaleAbs(image)

        logging.Debug.debug("binarize (theshohold)")

        return image


    #                     ####
    # Finds the pattern:  #  #  in the image.
    #                    #  #
    @staticmethod
    def find_open_square(image):
        logging.Debug.debug("Entered findOpenSquare")

        # cinarize the image
        image = ImageProcessing.binarizeImageInv(image)

        # find the contours
        im2, contours, hierarchy = cv.findContours(
            image,
            cv.RETR_TREE,
            cv.CHAIN_APPROX_SIMPLE
        )

        logging.Debug.debug("Find conturs")

        if not len(contours):
            return None

        # largest area
        chosen_contour = contours[0]
        max_size = cv.contourArea(chosen_contour)
        for contour in contours:
            curr_size = cv.contourArea(contour)
            if curr_size > max_size:
                max_size = curr_size
                chosen_contour = contour

        logging.Debug.debug("Largest Area")

        # Find bounding square
        point_list = cv.boxPoints(cv.minAreaRect(chosen_contour))
        point_list = np.int0(point_list)

        logging.Debug.debug("Find Bounding square")

        # Reorder square points into CW order
        centroid = np.mean(point_list, axis=0)
        list(point_list).sort(
            key=lambda x : math.atan2(
                -(x - centroid)[1],
                (x - centroid)[0]
            )
        )

        logging.Debug.debug("Reorder points")



        # find the missing edge
        max_dist = -1
        max_dist_index = -1
        for point_index in range(len(point_list)):
            mid_point = (
                point_list[point_index]
                + point_list[(point_index + 1) % len(point_list)]
            ) / 2

            min_dist = min([
                np.linalg.norm(mid_point - point)
                for point in chosen_contour
            ])

            if min_dist > max_dist:
                max_dist = min_dist
                max_dist_index = point_index


        # rotate by max_dist_index
        np.roll(point_list, max_dist_index)

        logging.Debug.debug("Find missing edge and finish findOpenSquare")

        return point_list
