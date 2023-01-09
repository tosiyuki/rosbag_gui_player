#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import threading

import rospy
import numpy as np
import cv2
from PyQt5.QtWidgets import *
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import Qt
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from Player import Player


MAX_PLAY_SPEED = 32.0
MIN_PLAY_SPEED = 0.03125


class PlayerWidget(QWidget):
    def __init__(self):
        super(PlayerWidget, self).__init__()

        self._bag = None
        self.player = Player()

        self.play_flag = False
        self.slider_thread = None
        self.topic_widget = None
        self._prev_slider_value = 0
        self._image_sub = None

        # widget layout
        self.player_layout = QGridLayout()
        self.setLayout(self.player_layout)

        # compressed image selector
        self.image_combobox = QtWidgets.QComboBox()
        self.image_combobox.currentTextChanged.connect(self.image_combobox_changed)
        self.player_layout.addWidget(self.image_combobox, 0, 1)

        # image view
        self.view = QGraphicsView()
        self.view.setFixedSize(820, 380) # TODO config
        self.scene = QGraphicsScene()
        self.view.setScene(self.scene)
        self.player_layout.addWidget(self.view, 1, 1)

        # play slider
        self.slider = QSlider(Qt.Orientation.Horizontal, self)
        self.slider.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.slider.setMaximum(1000)
        self.slider.setMinimum(0)
        self.slider.setValue(0)
        self.play_restart_flag = False
        self.slider.sliderPressed.connect(self.slider_pressed)
        self.slider.sliderReleased.connect(self.slider_released)
        self.player_layout.addWidget(self.slider, 2, 1)

        # label
        self.played_time_label = QLabel(self._time_to_str(0.0))
        self.leave_time_label = QLabel(self._time_to_str(self.player.duration))
        self.play_speed_label = QLabel(f"Play Speed : {str(self.player.play_speed)}")
        self.player_layout.addWidget(self.played_time_label, 3, 0)
        self.player_layout.addWidget(self.leave_time_label, 3, 2)
        self.player_layout.addWidget(self.play_speed_label, 4, 1, Qt.AlignmentFlag.AlignCenter)

        # button
        self.play_slow_btn = QPushButton()
        self.play_slow_btn.setToolTip("Slow")
        self.play_slow_btn.clicked.connect(self.on_slow)
        slow_pixmapi = getattr(QStyle, "SP_MediaSeekBackward")
        slow_icon = self.style().standardIcon(slow_pixmapi)
        self.play_slow_btn.setIcon(slow_icon)
        self.player_layout.addWidget(self.play_slow_btn, 2, 0, Qt.AlignmentFlag.AlignCenter)

        self.play_fast_btn = QPushButton()
        self.play_fast_btn.setToolTip("Fast")
        self.play_fast_btn.clicked.connect(self.on_fast)
        fast_pixmapi = getattr(QStyle, "SP_MediaSeekForward")
        fast_icon = self.style().standardIcon(fast_pixmapi)
        self.play_fast_btn.setIcon(fast_icon)
        self.player_layout.addWidget(self.play_fast_btn, 2, 2, Qt.AlignmentFlag.AlignCenter)

        self.play_stop_btn = QPushButton()
        self.play_stop_btn.setFixedSize(60, 30)
        self.play_stop_btn.setToolTip("Play")
        self.play_stop_btn.clicked.connect(self.on_play_stop)
        play_pixmapi = getattr(QStyle, "SP_MediaPlay")
        self.play_icon = self.style().standardIcon(play_pixmapi)
        stop_pixmapi = getattr(QStyle, "SP_MediaStop")
        self.stop_icon = self.style().standardIcon(stop_pixmapi)
        self.play_stop_btn.setIcon(self.play_icon)
        self.player_layout.addWidget(self.play_stop_btn, 3, 1, Qt.AlignmentFlag.AlignCenter)

        # update image
        scene_timer = QtCore.QTimer(self)
        scene_timer.timeout.connect(self.update_scene)
        scene_timer.start(10)
        self.pixmap = None

    @property
    def bag(self):
        pass

    @bag.setter
    def bag(self, bag):
        self._bag = bag
        self._set_combbox_item()
        self.player.bag = self._bag
        self.player.create_publisher()

    def read_rosbag_info(self, rosbag_path):
        """get rosbag info and set label

        Args:
            rosbag_path (string): rosbag path
        """
        self.player.read_rosbag_info(rosbag_path)

        # init parameter
        self.slider.setValue(0)
        self.played_time_label.setText(self._time_to_str(0.0))
        self.leave_time_label.setText(self._time_to_str(self.player.duration))
        self.player.now_time = 0.0

    def on_play_stop(self):
        """rosbag play or stop
        if play start all threads, elif stop stop all threads
        """
        if self.play_flag:
            # stoping
            self.player.stop()
            self.play_stop_btn.setIcon(self.play_icon)
            self.play_flag = False
        else:
            # playing
            topics = self.topic_widget.get_check_topics()

            if len(topics) == 0:
                QtWidgets.QMessageBox.critical(self, "Error", "Please select at least one topic")
                return

            start_time_reset = False
            if self.slider.value() == 1000:
                start_time_reset = True

            self.player.play(topics, start_time_reset)
            self.play_stop_btn.setIcon(self.stop_icon)
            self.play_flag = True

            self.slider_thread = threading.Thread(target=self._move_slider)
            self.slider_thread.start()

    def on_slow(self):
        """play speed half
        """
        tmp_play_flag = self.play_flag

        if self.play_flag:
            # if rosbag is playing, all threads restart
            self.player.stop()
            self.play_flag = False

        self.player.play_speed*=0.5
        self.play_speed_label.setText(f"Play Speed : {str(self.player.play_speed)}")

        if self.player.play_speed <= MIN_PLAY_SPEED:
            self.play_slow_btn.setEnabled(False)
        self.play_fast_btn.setEnabled(True)

        if tmp_play_flag:
            self.on_play_stop()

    def on_fast(self):
        """play speed double
        """
        tmp_play_flag = self.play_flag

        if self.play_flag:
            # if rosbag is playing, all threads restart
            self.player.stop()
            self.play_flag = False

        self.player.play_speed*=2
        self.play_speed_label.setText(f"Play Speed : {str(self.player.play_speed)}")
        
        if self.player.play_speed >= MAX_PLAY_SPEED:
            self.play_fast_btn.setEnabled(False)
        self.play_slow_btn.setEnabled(True)

        if tmp_play_flag:
            self.on_play_stop()

    def slider_pressed(self):
        """press slider event
        """
        if self.play_flag:
            # if rosbag is playing, all threads stop
            self.player.stop()
            self.play_flag = False
            self.play_restart_flag = True

    def slider_released(self):
        """release slider event
        """
        value = self.slider.value()

        # calculate played time by duration and slider value
        now_time = (self.player.duration / 1000) * value
        leave_time = self.player.duration - now_time
        played_time = self.player.duration - leave_time
        
        # update label
        self.played_time_label.setText(self._time_to_str(played_time))
        self.leave_time_label.setText(self._time_to_str(leave_time))

        self.player.now_time = now_time
        if self._prev_slider_value < value:
            # if slider is moved right, pass
            pass
        else:
            if self.play_flag:
                # if slider is moved left, all threads restart
                self.on_play_stop()
                if value != 1000:
                    self.on_play_stop()

        self._prev_slider_value = value

        if self.play_restart_flag:
            self.on_play_stop()
            self.play_restart_flag = False

    def _move_slider(self):
        while self.play_flag:
            slider_value = (self.player.now_time / self.player.duration) * 1000

            if slider_value > 1000:
                slider_value = 1000

            self.slider.setValue(slider_value)

            # set played label
            leave_time = self.player.duration - self.player.now_time
            played_time = self.player.duration - leave_time
            self.played_time_label.setText(self._time_to_str(played_time))
            self.leave_time_label.setText(self._time_to_str(leave_time))
            
            if slider_value == 1000:
                # play rosbag endding process
                self.leave_time_label.setText(self._time_to_str(0))
                self.on_play_stop()
                return

            time.sleep(0.1)

    def quit(self):
        """this instance end process
        """
        if self.play_flag:
            # if rosbag is playing, all threads stop
            self.on_play_stop()
            self.slider_thread.join()

    def _time_to_str(self, time):
        hour = int(time / 3600)
        time %= 3600
        minute = int(time / 60)
        time %= 60
        second = int(time)
        return "{}:{:02}:{:02}".format(hour, minute, second)

    def _set_combbox_item(self):
        self.image_combobox.clear()

        topic_list = self._bag.get_type_and_topic_info()[1]
        for key, value in topic_list.items():
            if value[0] == "sensor_msgs/CompressedImage":
                self.image_combobox.addItem(key)

        if self.image_combobox.count != 0:
            self.image_combobox_changed(str(self.image_combobox.currentText()))

    def call_back_image(self, compressed_image: CompressedImage):
        """recv compressed image and change qt_image

        Args:
            compressed_image (CompressedImage): receive topic
        """
        try:
            np_arr = np.fromstring(compressed_image.data, np.uint8)
            cv2_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError:
            print("CvBridgeError")
            return

        # cv image to qt pixmap
        cv2_image = cv2.resize(cv2_image, dsize=(800, 360))
        height, width, dim = cv2_image.shape
        bytesPerLine = dim * width
        qt_image = QtGui.QImage(cv2_image, width, height, bytesPerLine, QtGui.QImage.Format_RGB888).rgbSwapped()
        self.pixmap = QtGui.QPixmap.fromImage(qt_image)

    def update_scene(self):
        if self.pixmap is None:
            return

        self.scene.clear()
        self.scene.addPixmap(self.pixmap)

    def image_combobox_changed(self, topic):    
        if self._image_sub is not None:
            self._image_sub.unregister()

        if topic == "":
            return
            
        self._image_sub = rospy.Subscriber(topic, CompressedImage, self.call_back_image, queue_size=10)