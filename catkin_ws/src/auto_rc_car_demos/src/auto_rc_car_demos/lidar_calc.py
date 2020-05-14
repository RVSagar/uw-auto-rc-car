

class LidarCalc:
    def __init__(self):
        pass

    def lidar_data_too_close(self, scan, th1, th2, min_dist):
        # Returns fraction of points between angles [th1, th2]
        #  that are closer than min_dist
        if th2 < th1:
            temp = th1
            th1 = th2
            th2 = temp
        
        th1 = max(th1, scan.angle_min)
        th2 = min(th2, scan.angle_max)


        ind_start = int((th1 - scan.angle_min) / scan.angle_increment)
        ind_end = int((th2 - scan.angle_min) / scan.angle_increment)

        meas = scan.ranges[ind_start:ind_end]
        total = len(meas)
        meas = [m for m in meas if np.isfinite(m)]

        if len(meas) == 0:
            return 0

        num_too_close = 0.0
        for m in meas:
            if m < min_dist:
                num_too_close = num_too_close + 1

        return float(num_too_close) / total