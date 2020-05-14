import cv2
import math
import numpy as np

class BasicCameraCalc:
    def __init__(self):
        pass

    def detect_sign(self, img):
        # Return:
        ## 'r'/'g'/'b'/'n' for color detected
        ## dominance of that color, or 0 if none
        rows, cols, _ = img.shape
        midr = rows/2
        midc = cols/2

        # Find circles in image
        grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        grey = grey[0:midr, midc:]
        circles = cv2.HoughCircles(grey, cv2.HOUGH_GRADIENT, 1, 25, param1=30, param2=18)

        if circles is None:
            return 'n', 0

        circles = np.round(circles[0, :]).astype("int")
        # loop over the (x, y) coordinates and radius of the circles
        for (x0, y0, r) in circles:
            if r < 8:
                print("too small")
                continue

            x = midc+x0
            y = midr+y0
            sr = int(1.0/math.sqrt(2.0) * r)

            # Bounding box coordinates
            i_low = x - sr
            i_high = x + sr
            if i_low < 0 or i_high >= cols:
                #print("i (%d, %d) out of range" % (i_low, i_high))
                continue

            j_low = y0 - sr
            j_high = y0 + sr
            if j_low < 0 or j_high >= rows:
                #print("j (%d, %d) out of range" % (j_low, j_high))
                continue

            #cv2.circle(img, (x , y0), r, (0, 255, 0), 4)
            #cv2.rectangle(img, (x - 5, y0 - 5), (x + 5, y0 + 5), (0, 128, 255), -1)

            # Sum up colored pixels in bounding box
            color_acc = [0,0,0]
            count = 0.0
            for i in range(i_low, i_high):
                for j in range(j_low, j_high):
                    color = img[j, i]
                    #print(color)
                    color_acc = color_acc + color
                    count = count + 1.0
            color_acc = color_acc / count
            #print("avg_color: {}".format(color_acc))

            b = color_acc[0]
            g = color_acc[1]
            r = color_acc[2]

            tot = float(b + g + r)

            thresh = 0.5
            r = r / tot
            g = g / tot

            if r > thresh:
                return 'r', r

            if g > thresh:
                return 'g', g

        return 'n', 0

    def get_black_centroid(self, img, scaled_by):
        rows, cols, _ = img.shape
        count = 0
        totX = 0
        
        threshold = 20
        num_thresh = 30 
        drop_bottom_rows = 20

        for y in range(int(rows/2.5), rows-drop_bottom_rows):
            for x in range(0, cols):
                b,g,r = img[y, x]
                
                if b < threshold and g < threshold and r < threshold:
                    count += 1
                    totX += x

        if totX < num_thresh:
            centroid_x = int(cols / 2)
        else:
            centroid_x = int(totX / (count + 1))
        
        cv2.line(img, (centroid_x, 0), (centroid_x, rows), (0, 0, 255), 2)
        return int(centroid_x / scaled_by)