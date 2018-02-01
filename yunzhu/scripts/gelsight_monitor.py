# system
import sys
import numpy as np
import os
import time
import datetime

# opencv
import cv2

# spartan
import spartan.utils.utils as spartanUtils


class GelsightMonitor(object):

    def __init__(self, idx, num_record):
        self.num_record = int(num_record)
        self.idx = idx
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 800)
        self.cap.set(4, 600)
        # self.cap.set(cv2.CAP_PROP_FPS, 20)

        print self.cap.get(cv2.CAP_PROP_FPS)

        if(self.cap.isOpened() == False):
            print "Unable to read GelSight feed"

        self.rec_dir_name = os.path.join(spartanUtils.getSpartanSourceDir(),
                                         'yunzhu', 'data', 'gelsight_rec',
                                         'gelsight_rec_' + self.idx)
        os.system("mkdir -p " + self.rec_dir_name)

    def record(self):
        rec_name = os.path.join(self.rec_dir_name, 'gelsight_rec_' +
                                str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")))
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(rec_name + '.avi', fourcc, 20.0, (800, 600))

        print 'gelsight', str(datetime.datetime.now())

        for i in xrange(self.num_record):
            print str(datetime.datetime.now())
            ret, frame = self.cap.read()
            self.out.write(frame)

        print 'gelsight', str(datetime.datetime.now())


    def clean(self):
        self.cap.release()
        self.out.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    gelsight_monitor = GelsightMonitor(sys.argv[1], sys.argv[2])
    time.sleep(0.8)
    gelsight_monitor.record()
    gelsight_monitor.clean()
    time.sleep(0.5)

