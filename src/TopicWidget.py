#!/usr/bin/python
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import *
from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt


class TopicWidget(QWidget):
    def __init__(self, rosbag_path, bag, msg=None):
        super(TopicWidget, self).__init__()

        self.bag = bag
        self.msg = msg

        self.topic_list = []
        self.checkboxs=[]
        self.prev_check_topic = []

        # widget layout
        topic_layout = QVBoxLayout()
        self.setLayout(topic_layout)

        # label
        self.label_layout = QGridLayout()
        self.msg_label = QLabel(msg)
        self.file_path_label = QLabel(f'rosbag path : {rosbag_path}')
        self.file_path_label.setToolTip(rosbag_path)
        self.label_layout.addWidget(self.msg_label, 0, 0)
        self.label_layout.addWidget(self.file_path_label, 1, 0)

        # button
        self.button_layout = QGridLayout()
        self.button_layout.setContentsMargins(5, 5, 5, 5)
        self.all_check_btn = QPushButton("All Check")
        self.all_check_btn.setFixedWidth(100)
        self.all_check_btn.clicked.connect(self.on_all_check)
        self.button_layout.addWidget(self.all_check_btn, 0, 0, Qt.AlignmentFlag.AlignLeft)

        self.all_uncheck_btn = QPushButton("All Uncheck")
        self.all_uncheck_btn.setFixedWidth(100)
        self.all_uncheck_btn.clicked.connect(self.on_all_uncheck)
        self.button_layout.addWidget(self.all_uncheck_btn, 1, 0, Qt.AlignmentFlag.AlignLeft)

        # topics
        topic_checkbox_widget = self.load_topics()
        
        topic_layout.addLayout(self.label_layout)
        topic_layout.addLayout(self.button_layout)
        topic_layout.addLayout(topic_checkbox_widget)

    def load_topics(self, filepath=None):
        """load rosbag and set topic list layout

        Args:
            filepath (string, optional): rosbag file path. Defaults to None.

        Returns:
            QVBoxLayout: topic list
        """
        topic_checkbox_widget = QVBoxLayout()

        if self.bag is None:
            return topic_checkbox_widget

        if filepath is not None:
            self.file_path_label.setText(f'rosbag path : {filepath}')
            self.file_path_label.setToolTip(filepath)
        
        topic_list = self.bag.get_type_and_topic_info()[1].keys()
        if len(topic_list) < 1:
            print("Please select a bag file")
            QtWidgets.QMessageBox.critical(self, "Error", "Please select a bag file")
            return

        self.checkboxs.clear()
        self.topic_list = topic_list

        for select in self.topic_list:
            checkbox = QCheckBox(select)

            if select in self.prev_check_topic:
                checkbox.setChecked(True)

            topic_checkbox_widget.addWidget(checkbox)
            self.checkboxs.append(checkbox)

        return topic_checkbox_widget

        
    def on_all_check(self):
        """checked all topics
        """
        for checkbox in self.checkboxs:
            checkbox.setChecked(True)

    def on_all_uncheck(self):
        """unchecked all topics
        """
        for checkbox in self.checkboxs:
            checkbox.setChecked(False)

    def get_check_topics(self):
        """get checked topics name list

        Returns:
            list(string): checed topics name list
        """
        topics = []
        
        for (checkbox, select) in zip(self.checkboxs, self.topic_list):
            if checkbox.isChecked():
                topics.append(select)

        return topics

    def check_topics(self, topics):
        """check topics

        Args:
            topics (list(string)): you want to check topics name list
        """
        for (checkbox, select) in zip(self.checkboxs, self.topic_list):
            if select in topics:
                checkbox.setChecked(True)