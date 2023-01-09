#!/usr/bin/python
# -*- coding:utf-8 -*-

from PyQt5.QtWidgets import *
import sys


class FileWindow(QWidget):
    def __init__():
        pass

    def QuitApp(self):
        QApplication.quit()

    @classmethod
    def get_file_path(self, caption="Open File", filefilter="",isApp=False):
        if not isApp:
            app = QApplication(sys.argv)
        file_path = QFileDialog.getOpenFileName(caption=caption,filter=filefilter)

        return file_path

    @classmethod
    def save_file(self, caption="Save File", filefilter="", isApp=False):
        if not isApp:
            app = QApplication(sys.argv)
        file_path = QFileDialog.getSaveFileName(caption=caption, filter=filefilter)

        return file_path


if __name__ == '__main__':
    filePath=FileWindow.get_file_path(caption=u"Select files",filefilter="*py")
    print(filePath)
