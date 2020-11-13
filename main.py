import sys
from PySide2.QtWidgets import QCheckBox,QApplication, QLabel, QDialog, QLineEdit, QPushButton, QVBoxLayout, QMessageBox, QHBoxLayout, QComboBox, QFormLayout
import pyautogui as pyau
from PySide2.QtCore import QTimer
from PySide2 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
import numpy as np
from driver import *
import threading
import queue
from tracker import Tracker
from sklearn.cluster import KMeans
from rect import RectItem
from kalman_filter import KalmanFilter
import hdbscan
from sklearn.preprocessing import StandardScaler
import autopy
from gesture import Gesture

class Form(QDialog):
    def __init__(self, parent=None):
        super(Form, self).__init__(parent)

        self.setWindowTitle("Lidar V1.0 beta")
        self.SCREEN_SIZE = pyau.size()
        self.SCREEN_SCALE = 0.4

        self.cbbox_port = QComboBox(self)
        self.cbbox_port.addItems(
            ["/dev/tty.usbserial-0001", "COM3", "/dev/tty.USB0"])
        self.edit_screen_size = QLineEdit("Screen size...")
        self.edit_screen_scale = QLineEdit("Screen scale...")
        self.edit_mouse = QLineEdit("Screen scale...")
        self.button_update = QPushButton("Update")
        self.button_start = QPushButton("Start")
        self.checkbox_running = QCheckBox("Is running")
        self.checkbox_running.setEnabled(False)
        # ===========================

        layout_form = QFormLayout()

        layout_form.addRow("Port", self.cbbox_port)
        layout_form.addRow("Sreen size:", self.edit_screen_size)
        layout_form.addRow("Screen scale:", self.edit_screen_scale)
        layout_form.addRow("Mouse:", self.edit_mouse)
        layout_form.addRow("Gesture state:",self.checkbox_running)
        layout_form.addRow(self.button_start)
        layout_form.addRow(self.button_update)
        self.pw = pg.PlotWidget(name="mygrapg")

        layout = QHBoxLayout()
        layout.addLayout(layout_form)
        self.setLayout(layout)
        layout.addWidget(self.pw)

        # plotting
        self.pw.setXRange(1000, -1000, padding=0)
        self.pw.setYRange(1000, -1000, padding=0)
        self.pw.invertX()
        self.pw.invertY()
        self.pw.scene().sigMouseClicked.connect(self.mouse_clicked)

        self.plotAllDataItem = self.pw.plot([], pen=None, symbolBrush=(
            255, 255, 0), symbolSize=3, symbolPen=None)

        self.plotTouchItem = self.pw.plot([], pen=None, symbolBrush=(
            255, 0, 0), symbolSize=3, symbolPen=None)

        self.plotCenterItem = self.pw.plot([], pen=None, symbolBrush=(
            255, 100, 100), symbolSize=5, symbolPen=None,symbol="+")

        self.pw.plot([0], pen=None, symbolBrush=(255, 0, 0),
                     symbolSize=5, symbolPen=None, symbol="+")

        self.plot = self.pw.plot()
        # timer
        self.timer = QTimer(self)
        self.timer.setInterval(10)  # in milliseconds
        self.timer.timeout.connect(self.loop)
        self.button_start.clicked.connect(self.change_state_lidar)
        self.button_update.clicked.connect(self.change_value)
        #
        self.lidar = None
        self.touchable_area_position = [-1, -1]
        self.lidar_running = False
        self.touchable_rect = None
        self.current_mouse = queue.Queue(maxsize=100)
        self.is_exist = False
        self.is_gesture = False
        self.tracker = None
        self.clear_tracker()
        #threading and queue

        self.queue_cluster = queue.Queue(maxsize=360)
        self.thread_cluster = threading.Thread(
            target=self.cluster_proccessing, args=())
        self.thread_filtering = threading.Thread(
            target=self.filtering, args=())
        self.thread_cluster.start()
        self.thread_filtering.start()



        self.two_hand_detect= False
        self.gesture = Gesture()


        self.edit_screen_scale.setText(str(self.SCREEN_SCALE))
        self.edit_screen_size.setText("%dx%d" % (
            self.SCREEN_SIZE[0], self.SCREEN_SIZE[1]))
        self.edit_mouse.setText("%0.3f x %0.3f" % (
            self.touchable_area_position[0], self.touchable_area_position[1]))
        self.change_value()

    def change_value(self):
        try:
            self.SCREEN_SCALE = float(self.edit_screen_scale.text())
            screen_size = self.edit_screen_size.text().split("x")
            self.SCREEN_SIZE = [int(screen_size[0]),int(screen_size[1])]
            if self.touchable_rect:
                self.pw.removeItem(self.touchable_rect)
            self.touchable_rect = RectItem(QtCore.QRectF( self.touchable_area_position[0], self.touchable_area_position[1], self.SCREEN_SIZE[0]*self.SCREEN_SCALE, self.SCREEN_SIZE[1]*self.SCREEN_SCALE))
            self.pw.addItem(self.touchable_rect)
        except Exception as error:
            print(str(e))

    def filtering(self):
        previous_mouse_pos = []
        while not self.is_exist:
            time.sleep(0.1)

            if not self.current_mouse.empty():
                current_mouse = self.current_mouse.get()
                self.tracker.Update(current_mouse)

                if len(current_mouse) ==2:
                    continue

                now_x = []
                now_y = []
                for i in range(len(self.tracker.tracks)):
                    if (len(self.tracker.tracks[i].trace) > 1):
                        for j in range(len(self.tracker.tracks[i].trace)):
                            x1 = self.tracker.tracks[i].trace[j][0][0]
                            y1 = self.tracker.tracks[i].trace[j][0][1]
                            now_x.append(x1)
                            now_y.append(y1)
                self.plotCenterItem.setData(now_x, now_y)

                # one hand detect
                if len(current_mouse) ==1:
                    if len(now_x) > 0:
                        X = int(
                            (self.touchable_area_position[0]-now_x[-1])/self.SCREEN_SCALE+self.SCREEN_SIZE[0])
                        Y = int(
                            (now_y[-1] - self.touchable_area_position[1])/self.SCREEN_SCALE)
                        if self.is_gesture:
                            pyau.moveTo(X,Y)

    def two_hand_gesture(self):
        first_hand_on_left = False
        if len(self.tracker.tracks) ==2:
            if len(self.tracker.tracks[0].trace) > 2 and len(self.tracker.tracks[1].trace) > 2:
                first_hand = self.tracker.tracks[0].trace
                second_hand = self.tracker.tracks[1].trace
                first_hand_vector = [first_hand[-1][0][0]-first_hand[0][0][0],first_hand[-1][0][1]-first_hand[0][0][1]]
                second_hand_vector = [second_hand[-1][0][0]-second_hand[0][0][0],second_hand[-1][0][1]-second_hand[0][0][1]]
                # check first_hand x > second_hand x then first hand on rightside
                if first_hand[0][0][0] < second_hand[0][0][0]:
                    first_hand_on_left = True

                if first_hand_vector[0] * second_hand_vector[0] < 0:
                    if first_hand_on_left:
                        if first_hand_vector[0] < 0 :
                            self.gesture.zoom_in()
                        else:
                            self.gesture.zoom_out()
                    else:
                        if second_hand_vector[0] < 0:
                            self.gesture.zoom_in()
                        else:
                            self.gesture.zoom_out()
                # same direction
                else:
                    if first_hand_vector[0] >0:
                        self.gesture.slide_move_right()
                    else:
                        self.gesture.slide_move_left()

    def clear_tracker(self):
        if self.tracker != None and self.is_gesture == True:
            self.two_hand_gesture()
        self.tracker = Tracker(200, 2, 3, 20)

    def cluster_proccessing(self):
        while not self.is_exist:
            try:
                time.sleep(0.1)
                if not self.queue_cluster.empty():
                    centers = []
                    X = self.queue_cluster.get()
                    centers = self.detector(X)
                    if self.distance(centers[0][0], centers[0][1], centers[1][0], centers[1][1]) < 70:
                        centers = [np.array([np.mean(centers[:, 0]), np.mean(centers[:, 1])])]
                    self.current_mouse.put(centers)
            except Exception as e:
                print(str(e))

    def detector(self, X):
        scaler = StandardScaler()
        X = np.array(X)
        clusters = KMeans(algorithm='auto', init='k-means++',
                          max_iter=300, n_clusters=2, random_state=0, tol=0.0001, verbose=0).fit(X)
        return np.array(clusters.cluster_centers_)

    def loop(self):
        dataset = []
        X = []
        last_deg = 0
        count = 0
        for deg,dis in self.lidar.poll():
            #dis = ukf.smooth([dis])[0][0]
            x = dis*np.sin(np.deg2rad(deg))
            y = dis*np.cos(np.deg2rad(deg))
            if self.in_rect(x, y):
                X.append([x,y])
            else:
                dataset.append([x, y])

        if len(X) != 0:
            X = np.array(X)
            self.setData(X)
            self.queue_cluster.put(X)

        else:
            self.clear_tracker()
            self.plotAllDataItem.clear()
            self.plotCenterItem.clear()

        dataset =np.array(dataset)
        self.setPoint(dataset)

    def get_cluster(self, X, clusters):
        return [X[np.where(clusters.labels_ == i)] for i in range(clusters.labels_.max())]

    def in_rect(self, x, y):
        return x >= self.touchable_area_position[0] and x <= self.touchable_area_position[0]+self.SCREEN_SIZE[0]*self.SCREEN_SCALE and y >= self.touchable_area_position[1] and y <= self.touchable_area_position[1]+self.SCREEN_SIZE[1]*self.SCREEN_SCALE

    def setData(self, data):
        self.plotAllDataItem.setData(data[:, 0], data[:, 1])

    def setPoint(self, data):
        self.plotTouchItem.setData(data[:, 0], data[:, 1])

    def setCenter(self, data):
        self.plotCenterItem.setData(data[:, 0], data[:, 1])

    def distance(self, x1, y1, x2, y2):
        return np.sqrt((x1-x2)**2 + (y1-y2)**2)

    def closeEvent(self, event):
        self.is_exist = True
        time.sleep(0.5)
        self.lidar.stop()
        event.accept()

    def keyPressEvent(self, e):
        if e.key() == QtCore.Qt.Key_Escape:
            self.is_gesture = not self.is_gesture
            if self.is_gesture:
                self.checkbox_running.setCheckState(QtCore.Qt.Checked)
            else:
                self.checkbox_running.setCheckState(QtCore.Qt.Unchecked)

    def change_state_lidar(self):
        if self.lidar_running:
            self.lidar.stop()
            self.button_start.setText("Start")
            self.lidar_running = False
            self.timer.stop()
        else:
            self.lidar = Lidar(str(self.cbbox_port.currentText()))
            self.lidar.init()
            self.lidar_running = True
            self.timer.start()
            self.button_start.setText("Stop")

    # set position of touchable area
    def mouse_clicked(self, evt):
        pos = evt.pos()
        mousePoint = self.pw.plotItem.vb.mapSceneToView(pos).toTuple()
        self.touchable_area_position =mousePoint
        self.edit_mouse.setText("%0.3f x %0.3f" % ( self.touchable_area_position[0], self.touchable_area_position[1]))
        self.change_value()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    form = Form()
    form.show()
    sys.exit(app.exec_())
