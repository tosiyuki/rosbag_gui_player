#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys

from PyQt5.QtWidgets import *
from PyQt5 import QtWidgets
import rosbag
import rospy

from FileWindow import FileWindow
from TopicWidget import TopicWidget
from PlayerWidget import PlayerWidget


class MainWindow(QMainWindow):
    def __init__(self, title="rosbag player", msg="",app=None):
        super().__init__()

        self.msg = msg
        self.app = app

        self.rosbag_path = ""
        self.bag = None
        self.scroll_area = None

        # init widget
        self.player_widget = PlayerWidget()
        self.load_rosbag(title, True)
        self.show()

    def load_rosbag(self, title="", initialize=False):
        """load rosbag file

        Args:
            title (str, optional): main window title. Defaults to "".
            initialize (bool, optional): initialize flag. Defaults to False.
        """
        # stop all threads when rosbag is playing.
        self.player_widget.quit()

        checked_topic_list = []
        if self.rosbag_path != "":
            # get all checked topic name
            checked_topic_list = self.topic_widget.get_check_topics()

        file_paths = FileWindow.get_file_path(isApp=True,caption="Select bag file",filefilter="*bag")
        if file_paths[0] == "":
            return

        self.rosbag_path = file_paths[0]
        self.bag = rosbag.Bag(self.rosbag_path)
        self.player_widget.bag = self.bag
        self.player_widget.read_rosbag_info(self.rosbag_path)

        # init gui
        self._init_gui(title, initialize)
        self.topic_widget.check_topics(checked_topic_list)

    def _init_gui(self, title, initialize=True):
        # set ui component
        scroll_area = QScrollArea()
        inner = QWidget()
        box_layout = QVBoxLayout()

        self.topic_widget = TopicWidget(self.rosbag_path, self.bag, self.msg)
        self.player_widget.topic_widget = self.topic_widget
   
        # set layout
        box_layout.addWidget(self.topic_widget)
        box_layout.addWidget(self.player_widget)
        inner.setLayout(box_layout)
        scroll_area.setWidget(inner)
        self.setCentralWidget(scroll_area)

        if initialize:
            # window size and init position setting
            self.setGeometry(300, 300, 1000, 800)
            self.setWindowTitle(title)

            # create menu
            self.create_menu_bar()

    def create_menu_bar(self):
        """create menu bar
        """
        load_action = self._create_menu('&Load rosbag', self.load_rosbag, 'Ctrl+O', 'Load rosbag file')
        save_config_action = self._create_menu("&Save config", self.save_config, 'Ctrl+S', 'Save checkbox config')
        load_config_action = self._create_menu("&Load config", self.load_config, 'Ctrl+A', 'Load checkbox config')
        exit_action = self._create_menu('&Exit', self.quit, 'Ctrl+Q', 'Exit application')
        menubar = self.menuBar()

        file_menu = menubar.addMenu('&File')
        file_menu.addAction(load_action)
        file_menu.addAction(exit_action)

        config_menu = menubar.addMenu('&Config')
        config_menu.addAction(save_config_action)
        config_menu.addAction(load_config_action)

    def save_config(self):
        """save checkbox config
        """
        output_path = FileWindow.save_file(isApp=True, caption='Save config file', filefilter='')
        if output_path[0] == "":
            return

        checked_topics = self.topic_widget.get_check_topics()
        with open(output_path[0], 'w') as f:
            for topic in checked_topics:
                f.write(f'{topic}\n')

    def load_config(self):
        """load checkbox config
        """
        file_paths = FileWindow.get_file_path(isApp=True,caption='Select config file',filefilter='')
        if file_paths[0] == "":
            return

        checked_list = []
        with open(file_paths[0], 'r') as f:
            checked_list = f.readlines()

        if len(checked_list) == 0:
            return

        checked_list = [i.rstrip('\n') for i in checked_list]
        for (checkbox, select) in zip(self.topic_widget.checkboxs, self.topic_widget.topic_list):
            if select in checked_list:
                checkbox.setChecked(True)
            else:
                checkbox.setChecked(False)

    def closeEvent(self, event):
        """window close process

        Args:
            event (_type_): _description_
        """
        self.quit()

    def quit(self):
        self.player_widget.quit()
        self.app.quit()

    def _create_menu(self, name, connect, short_cut=None, tooltip=None):
        action = QAction(name, self)
        action.triggered.connect(connect)

        if short_cut is not None:
            action.setShortcut(short_cut) 

        if tooltip is not None:
            action.setStatusTip(tooltip)

        return action


def main():
    app = QtWidgets.QApplication(sys.argv)
    main_window = MainWindow(app=app,msg="Select topics which you want to publish", title="rosbag gui player")
    app.exec_()


if __name__ == '__main__':
    print("rosbag gui player start!!")
    rospy.init_node("rosbag_gui_player")
    main()
    print("rosbag gui player end!!")