# Form implementation generated from reading ui file '/home/jieun/Qt_Designer/Qt_Designer/imu_interface.ui'
#
# Created by: PyQt6 UI code generator 6.8.0
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt6 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(574, 694)
        self.widget = QtWidgets.QWidget(parent=Dialog)
        self.widget.setGeometry(QtCore.QRect(-10, -10, 621, 761))
        self.widget.setObjectName("widget")
        self.background = QtWidgets.QLabel(parent=self.widget)
        self.background.setGeometry(QtCore.QRect(-30, -70, 651, 811))
        self.background.setStyleSheet("background-color: white;")
        self.background.setText("")
        self.background.setObjectName("background")
        self.title = QtWidgets.QLabel(parent=self.widget)
        self.title.setGeometry(QtCore.QRect(20, 20, 151, 17))
        font = QtGui.QFont()
        font.setFamily("Ubuntu Sans")
        font.setPointSize(14)
        font.setBold(True)
        font.setItalic(True)
        font.setWeight(75)
        self.title.setFont(font)
        self.title.setObjectName("title")
        self.select_sensor = QtWidgets.QListView(parent=self.widget)
        self.select_sensor.setGeometry(QtCore.QRect(20, 60, 251, 71))
        self.select_sensor.setObjectName("select_sensor")
        self.widget_2 = QtWidgets.QWidget(parent=self.widget)
        self.widget_2.setGeometry(QtCore.QRect(20, 140, 251, 101))
        self.widget_2.setObjectName("widget_2")
        self.info = QtWidgets.QTextBrowser(parent=self.widget_2)
        self.info.setGeometry(QtCore.QRect(0, 0, 251, 101))
        font = QtGui.QFont()
        font.setFamily("URW Gothic [UKWN]")
        font.setBold(True)
        font.setWeight(75)
        self.info.setFont(font)
        self.info.setObjectName("info")
        self.verticalLayoutWidget = QtWidgets.QWidget(parent=self.widget_2)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(100, 0, 151, 65))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.info_layout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.info_layout.setContentsMargins(0, 0, 0, 0)
        self.info_layout.setObjectName("info_layout")
        self.portname = QtWidgets.QLabel(parent=self.verticalLayoutWidget)
        self.portname.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight|QtCore.Qt.AlignmentFlag.AlignTrailing|QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.portname.setObjectName("portname")
        self.info_layout.addWidget(self.portname)
        self.subscribenode = QtWidgets.QLabel(parent=self.verticalLayoutWidget)
        self.subscribenode.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight|QtCore.Qt.AlignmentFlag.AlignTrailing|QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.subscribenode.setObjectName("subscribenode")
        self.info_layout.addWidget(self.subscribenode)
        self.model = QtWidgets.QLabel(parent=self.verticalLayoutWidget)
        self.model.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight|QtCore.Qt.AlignmentFlag.AlignTrailing|QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.model.setObjectName("model")
        self.info_layout.addWidget(self.model)
        self.pushButton = QtWidgets.QPushButton(parent=self.widget_2)
        self.pushButton.setGeometry(QtCore.QRect(160, 70, 80, 25))
        self.pushButton.setObjectName("pushButton")
        self.stackedWidget = QtWidgets.QStackedWidget(parent=self.widget)
        self.stackedWidget.setGeometry(QtCore.QRect(20, 60, 551, 641))
        self.stackedWidget.setObjectName("stackedWidget")
        self.imu_page = QtWidgets.QWidget()
        self.imu_page.setObjectName("imu_page")
        self.graph_rpy = QtWidgets.QWidget(parent=self.imu_page)
        self.graph_rpy.setGeometry(QtCore.QRect(260, 0, 291, 201))
        self.graph_rpy.setStyleSheet("background-color : rgb(255, 134, 136)")
        self.graph_rpy.setObjectName("graph_rpy")
        self.graph_acc = QtWidgets.QWidget(parent=self.imu_page)
        self.graph_acc.setGeometry(QtCore.QRect(260, 210, 291, 201))
        self.graph_acc.setStyleSheet("background-color : rgb(255, 134, 136)")
        self.graph_acc.setObjectName("graph_acc")
        self.graph_gyro = QtWidgets.QWidget(parent=self.imu_page)
        self.graph_gyro.setGeometry(QtCore.QRect(260, 420, 291, 201))
        self.graph_gyro.setStyleSheet("background-color : rgb(255, 134, 136)")
        self.graph_gyro.setObjectName("graph_gyro")
        self.label_2 = QtWidgets.QLabel(parent=self.imu_page)
        self.label_2.setGeometry(QtCore.QRect(10, 190, 91, 17))
        font = QtGui.QFont()
        font.setFamily("URW Gothic [urw]")
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(parent=self.imu_page)
        self.label_3.setGeometry(QtCore.QRect(10, 350, 91, 17))
        font = QtGui.QFont()
        font.setFamily("URW Gothic [urw]")
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.widget_3dview = QtWidgets.QWidget(parent=self.imu_page)
        self.widget_3dview.setGeometry(QtCore.QRect(0, 370, 251, 251))
        self.widget_3dview.setObjectName("widget_3dview")
        self.textBrowser = QtWidgets.QTextBrowser(parent=self.imu_page)
        self.textBrowser.setGeometry(QtCore.QRect(0, 210, 251, 131))
        self.textBrowser.setObjectName("textBrowser")
        self.stackedWidget.addWidget(self.imu_page)
        self.lidar_page = QtWidgets.QWidget()
        self.lidar_page.setObjectName("lidar_page")
        self.stackedWidget.addWidget(self.lidar_page)
        self.comboBox = QtWidgets.QComboBox(parent=self.widget)
        self.comboBox.setGeometry(QtCore.QRect(30, 90, 111, 25))
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.label = QtWidgets.QLabel(parent=self.widget)
        self.label.setGeometry(QtCore.QRect(30, 70, 111, 17))
        self.label.setObjectName("label")
        self.background.raise_()
        self.stackedWidget.raise_()
        self.select_sensor.raise_()
        self.widget_2.raise_()
        self.title.raise_()
        self.comboBox.raise_()
        self.label.raise_()

        self.retranslateUi(Dialog)
        self.stackedWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.title.setText(_translate("Dialog", "Sensor Interface"))
        self.info.setHtml(_translate("Dialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'URW Gothic [UKWN]\'; font-size:9pt; font-weight:600; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Sans Serif\'; font-weight:400;\">Port name : </span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Sans Serif\'; font-weight:400;\">Subscribe node : </span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Sans Serif\'; font-weight:400;\">model : </span></p></body></html>"))
        self.portname.setText(_translate("Dialog", "none"))
        self.subscribenode.setText(_translate("Dialog", "none"))
        self.model.setText(_translate("Dialog", "none"))
        self.pushButton.setText(_translate("Dialog", "Refresh"))
        self.label_2.setText(_translate("Dialog", "IMU Data"))
        self.label_3.setText(_translate("Dialog", "3d viewer"))
        self.comboBox.setItemText(0, _translate("Dialog", "IMU - myahrs+"))
        self.comboBox.setItemText(1, _translate("Dialog", "LIDAR"))
        self.label.setText(_translate("Dialog", "Select sensor"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec())
