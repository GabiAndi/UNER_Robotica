# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'ui_mainwindowtrabajofinal.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

import images_rc

class Ui_MainWindowTrabajoFinal(object):
    def setupUi(self, MainWindowTrabajoFinal):
        if not MainWindowTrabajoFinal.objectName():
            MainWindowTrabajoFinal.setObjectName(u"MainWindowTrabajoFinal")
        MainWindowTrabajoFinal.resize(930, 710)
        MainWindowTrabajoFinal.setMinimumSize(QSize(930, 710))
        font = QFont()
        font.setFamily(u"Nimbus Sans")
        MainWindowTrabajoFinal.setFont(font)
        self.centralwidget = QWidget(MainWindowTrabajoFinal)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout = QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.groupBoxAcciones = QGroupBox(self.centralwidget)
        self.groupBoxAcciones.setObjectName(u"groupBoxAcciones")
        sizePolicy = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(1)
        sizePolicy.setHeightForWidth(self.groupBoxAcciones.sizePolicy().hasHeightForWidth())
        self.groupBoxAcciones.setSizePolicy(sizePolicy)
        self.horizontalLayout_3 = QHBoxLayout(self.groupBoxAcciones)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.horizontalSpacer_5 = QSpacerItem(800, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_3.addItem(self.horizontalSpacer_5)

        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalSpacer_5 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_4.addItem(self.verticalSpacer_5)

        self.pushButtonIniciar = QPushButton(self.groupBoxAcciones)
        self.pushButtonIniciar.setObjectName(u"pushButtonIniciar")

        self.verticalLayout_4.addWidget(self.pushButtonIniciar)

        self.pushButtonDetener = QPushButton(self.groupBoxAcciones)
        self.pushButtonDetener.setObjectName(u"pushButtonDetener")

        self.verticalLayout_4.addWidget(self.pushButtonDetener)

        self.verticalSpacer_6 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_4.addItem(self.verticalSpacer_6)


        self.horizontalLayout_3.addLayout(self.verticalLayout_4)

        self.horizontalSpacer_6 = QSpacerItem(800, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_3.addItem(self.horizontalSpacer_6)


        self.verticalLayout.addWidget(self.groupBoxAcciones)

        self.groupBoxVisualizacion = QGroupBox(self.centralwidget)
        self.groupBoxVisualizacion.setObjectName(u"groupBoxVisualizacion")
        sizePolicy1 = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(4)
        sizePolicy1.setHeightForWidth(self.groupBoxVisualizacion.sizePolicy().hasHeightForWidth())
        self.groupBoxVisualizacion.setSizePolicy(sizePolicy1)
        self.gridLayout = QGridLayout(self.groupBoxVisualizacion)
        self.gridLayout.setObjectName(u"gridLayout")
        self.groupBoxABB1 = QGroupBox(self.groupBoxVisualizacion)
        self.groupBoxABB1.setObjectName(u"groupBoxABB1")
        sizePolicy2 = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy2.setHorizontalStretch(1)
        sizePolicy2.setVerticalStretch(1)
        sizePolicy2.setHeightForWidth(self.groupBoxABB1.sizePolicy().hasHeightForWidth())
        self.groupBoxABB1.setSizePolicy(sizePolicy2)
        self.horizontalLayout = QHBoxLayout(self.groupBoxABB1)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalSpacer_2 = QSpacerItem(800, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer_2)

        self.widgetABB1 = QWidget(self.groupBoxABB1)
        self.widgetABB1.setObjectName(u"widgetABB1")
        sizePolicy3 = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.widgetABB1.sizePolicy().hasHeightForWidth())
        self.widgetABB1.setSizePolicy(sizePolicy3)
        self.widgetABB1.setMinimumSize(QSize(128, 128))
        self.widgetABB1.setMaximumSize(QSize(128, 128))
        self.widgetABB1.setStyleSheet(u"border-image: url(:/image/image/brazo.png) 0 0 0 0 stretch stretch;")

        self.horizontalLayout.addWidget(self.widgetABB1)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalSpacer_2 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_2.addItem(self.verticalSpacer_2)

        self.label_2 = QLabel(self.groupBoxABB1)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setAlignment(Qt.AlignCenter)

        self.verticalLayout_2.addWidget(self.label_2)

        self.labelABB1Accion = QLabel(self.groupBoxABB1)
        self.labelABB1Accion.setObjectName(u"labelABB1Accion")
        sizePolicy3.setHeightForWidth(self.labelABB1Accion.sizePolicy().hasHeightForWidth())
        self.labelABB1Accion.setSizePolicy(sizePolicy3)

        self.verticalLayout_2.addWidget(self.labelABB1Accion)

        self.label = QLabel(self.groupBoxABB1)
        self.label.setObjectName(u"label")
        self.label.setAlignment(Qt.AlignCenter)

        self.verticalLayout_2.addWidget(self.label)

        self.progressBarABB1 = QProgressBar(self.groupBoxABB1)
        self.progressBarABB1.setObjectName(u"progressBarABB1")
        self.progressBarABB1.setValue(0)

        self.verticalLayout_2.addWidget(self.progressBarABB1)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_2.addItem(self.verticalSpacer)


        self.horizontalLayout.addLayout(self.verticalLayout_2)

        self.widgetABB1Caja = QWidget(self.groupBoxABB1)
        self.widgetABB1Caja.setObjectName(u"widgetABB1Caja")
        sizePolicy3.setHeightForWidth(self.widgetABB1Caja.sizePolicy().hasHeightForWidth())
        self.widgetABB1Caja.setSizePolicy(sizePolicy3)
        self.widgetABB1Caja.setMinimumSize(QSize(128, 128))
        self.widgetABB1Caja.setMaximumSize(QSize(128, 128))
        self.widgetABB1Caja.setStyleSheet(u"border-image: url(:/image/image/caja_no.png) 0 0 0 0 stretch stretch;")

        self.horizontalLayout.addWidget(self.widgetABB1Caja)

        self.horizontalSpacer = QSpacerItem(800, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)


        self.gridLayout.addWidget(self.groupBoxABB1, 2, 0, 1, 1)

        self.groupBoxABBAutito = QGroupBox(self.groupBoxVisualizacion)
        self.groupBoxABBAutito.setObjectName(u"groupBoxABBAutito")
        sizePolicy4 = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy4.setHorizontalStretch(1)
        sizePolicy4.setVerticalStretch(0)
        sizePolicy4.setHeightForWidth(self.groupBoxABBAutito.sizePolicy().hasHeightForWidth())
        self.groupBoxABBAutito.setSizePolicy(sizePolicy4)
        self.verticalLayout_15 = QVBoxLayout(self.groupBoxABBAutito)
        self.verticalLayout_15.setObjectName(u"verticalLayout_15")
        self.verticalSpacer_7 = QSpacerItem(20, 156, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_15.addItem(self.verticalSpacer_7)

        self.groupBoxCarSensoresInfrarojos = QGroupBox(self.groupBoxABBAutito)
        self.groupBoxCarSensoresInfrarojos.setObjectName(u"groupBoxCarSensoresInfrarojos")
        self.horizontalLayout_6 = QHBoxLayout(self.groupBoxCarSensoresInfrarojos)
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.horizontalSpacer_10 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_6.addItem(self.horizontalSpacer_10)

        self.verticalLayout_16 = QVBoxLayout()
        self.verticalLayout_16.setObjectName(u"verticalLayout_16")
        self.label_15 = QLabel(self.groupBoxCarSensoresInfrarojos)
        self.label_15.setObjectName(u"label_15")
        self.label_15.setAlignment(Qt.AlignCenter)

        self.verticalLayout_16.addWidget(self.label_15)

        self.labelCarSensor0 = QLabel(self.groupBoxCarSensoresInfrarojos)
        self.labelCarSensor0.setObjectName(u"labelCarSensor0")

        self.verticalLayout_16.addWidget(self.labelCarSensor0)


        self.horizontalLayout_6.addLayout(self.verticalLayout_16)

        self.verticalLayout_17 = QVBoxLayout()
        self.verticalLayout_17.setObjectName(u"verticalLayout_17")
        self.label_16 = QLabel(self.groupBoxCarSensoresInfrarojos)
        self.label_16.setObjectName(u"label_16")
        self.label_16.setAlignment(Qt.AlignCenter)

        self.verticalLayout_17.addWidget(self.label_16)

        self.labelCarSensor1 = QLabel(self.groupBoxCarSensoresInfrarojos)
        self.labelCarSensor1.setObjectName(u"labelCarSensor1")

        self.verticalLayout_17.addWidget(self.labelCarSensor1)


        self.horizontalLayout_6.addLayout(self.verticalLayout_17)

        self.verticalLayout_18 = QVBoxLayout()
        self.verticalLayout_18.setObjectName(u"verticalLayout_18")
        self.label_17 = QLabel(self.groupBoxCarSensoresInfrarojos)
        self.label_17.setObjectName(u"label_17")
        self.label_17.setAlignment(Qt.AlignCenter)

        self.verticalLayout_18.addWidget(self.label_17)

        self.labelCarSensor2 = QLabel(self.groupBoxCarSensoresInfrarojos)
        self.labelCarSensor2.setObjectName(u"labelCarSensor2")

        self.verticalLayout_18.addWidget(self.labelCarSensor2)


        self.horizontalLayout_6.addLayout(self.verticalLayout_18)

        self.verticalLayout_19 = QVBoxLayout()
        self.verticalLayout_19.setObjectName(u"verticalLayout_19")
        self.label_18 = QLabel(self.groupBoxCarSensoresInfrarojos)
        self.label_18.setObjectName(u"label_18")
        self.label_18.setAlignment(Qt.AlignCenter)

        self.verticalLayout_19.addWidget(self.label_18)

        self.labelCarSensor3 = QLabel(self.groupBoxCarSensoresInfrarojos)
        self.labelCarSensor3.setObjectName(u"labelCarSensor3")

        self.verticalLayout_19.addWidget(self.labelCarSensor3)


        self.horizontalLayout_6.addLayout(self.verticalLayout_19)

        self.verticalLayout_20 = QVBoxLayout()
        self.verticalLayout_20.setObjectName(u"verticalLayout_20")
        self.label_19 = QLabel(self.groupBoxCarSensoresInfrarojos)
        self.label_19.setObjectName(u"label_19")
        self.label_19.setAlignment(Qt.AlignCenter)

        self.verticalLayout_20.addWidget(self.label_19)

        self.labelCarSensor4 = QLabel(self.groupBoxCarSensoresInfrarojos)
        self.labelCarSensor4.setObjectName(u"labelCarSensor4")

        self.verticalLayout_20.addWidget(self.labelCarSensor4)


        self.horizontalLayout_6.addLayout(self.verticalLayout_20)

        self.verticalLayout_21 = QVBoxLayout()
        self.verticalLayout_21.setObjectName(u"verticalLayout_21")
        self.label_20 = QLabel(self.groupBoxCarSensoresInfrarojos)
        self.label_20.setObjectName(u"label_20")
        self.label_20.setAlignment(Qt.AlignCenter)

        self.verticalLayout_21.addWidget(self.label_20)

        self.labelCarSensor5 = QLabel(self.groupBoxCarSensoresInfrarojos)
        self.labelCarSensor5.setObjectName(u"labelCarSensor5")

        self.verticalLayout_21.addWidget(self.labelCarSensor5)


        self.horizontalLayout_6.addLayout(self.verticalLayout_21)

        self.horizontalSpacer_9 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_6.addItem(self.horizontalSpacer_9)


        self.verticalLayout_15.addWidget(self.groupBoxCarSensoresInfrarojos)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.horizontalSpacer_8 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_4.addItem(self.horizontalSpacer_8)

        self.groupBoxCarSensoresUltrasonicos = QGroupBox(self.groupBoxABBAutito)
        self.groupBoxCarSensoresUltrasonicos.setObjectName(u"groupBoxCarSensoresUltrasonicos")
        self.horizontalLayout_5 = QHBoxLayout(self.groupBoxCarSensoresUltrasonicos)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.gridLayout_2 = QGridLayout()
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.verticalLayout_10 = QVBoxLayout()
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.label_10 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.label_10.setObjectName(u"label_10")
        self.label_10.setAlignment(Qt.AlignCenter)

        self.verticalLayout_10.addWidget(self.label_10)

        self.labelCarObsSensor15 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.labelCarObsSensor15.setObjectName(u"labelCarObsSensor15")

        self.verticalLayout_10.addWidget(self.labelCarObsSensor15)


        self.gridLayout_2.addLayout(self.verticalLayout_10, 1, 5, 1, 1)

        self.verticalLayout_13 = QVBoxLayout()
        self.verticalLayout_13.setObjectName(u"verticalLayout_13")
        self.label_13 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.label_13.setObjectName(u"label_13")
        self.label_13.setAlignment(Qt.AlignCenter)

        self.verticalLayout_13.addWidget(self.label_13)

        self.labelCarObsSensor16 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.labelCarObsSensor16.setObjectName(u"labelCarObsSensor16")

        self.verticalLayout_13.addWidget(self.labelCarObsSensor16)


        self.gridLayout_2.addLayout(self.verticalLayout_13, 3, 6, 1, 1)

        self.verticalLayout_7 = QVBoxLayout()
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.label_7 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setAlignment(Qt.AlignCenter)

        self.verticalLayout_7.addWidget(self.label_7)

        self.labelCarObsSensor12 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.labelCarObsSensor12.setObjectName(u"labelCarObsSensor12")

        self.verticalLayout_7.addWidget(self.labelCarObsSensor12)


        self.gridLayout_2.addLayout(self.verticalLayout_7, 0, 3, 1, 1)

        self.verticalLayout_14 = QVBoxLayout()
        self.verticalLayout_14.setObjectName(u"verticalLayout_14")
        self.label_14 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.label_14.setObjectName(u"label_14")
        self.label_14.setAlignment(Qt.AlignCenter)

        self.verticalLayout_14.addWidget(self.label_14)

        self.labelCarObsSensor1 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.labelCarObsSensor1.setObjectName(u"labelCarObsSensor1")

        self.verticalLayout_14.addWidget(self.labelCarObsSensor1)


        self.gridLayout_2.addLayout(self.verticalLayout_14, 4, 6, 1, 1)

        self.verticalLayout_8 = QVBoxLayout()
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.label_8 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setAlignment(Qt.AlignCenter)

        self.verticalLayout_8.addWidget(self.label_8)

        self.labelCarObsSensor13 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.labelCarObsSensor13.setObjectName(u"labelCarObsSensor13")

        self.verticalLayout_8.addWidget(self.labelCarObsSensor13)


        self.gridLayout_2.addLayout(self.verticalLayout_8, 0, 4, 1, 1)

        self.verticalLayout_12 = QVBoxLayout()
        self.verticalLayout_12.setObjectName(u"verticalLayout_12")
        self.label_12 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.label_12.setObjectName(u"label_12")
        self.label_12.setAlignment(Qt.AlignCenter)

        self.verticalLayout_12.addWidget(self.label_12)

        self.labelCarObsSensor8 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.labelCarObsSensor8.setObjectName(u"labelCarObsSensor8")

        self.verticalLayout_12.addWidget(self.labelCarObsSensor8)


        self.gridLayout_2.addLayout(self.verticalLayout_12, 4, 1, 1, 1)

        self.verticalLayout_9 = QVBoxLayout()
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.label_9 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setAlignment(Qt.AlignCenter)

        self.verticalLayout_9.addWidget(self.label_9)

        self.labelCarObsSensor14 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.labelCarObsSensor14.setObjectName(u"labelCarObsSensor14")

        self.verticalLayout_9.addWidget(self.labelCarObsSensor14)


        self.gridLayout_2.addLayout(self.verticalLayout_9, 0, 5, 1, 1)

        self.verticalLayout_11 = QVBoxLayout()
        self.verticalLayout_11.setObjectName(u"verticalLayout_11")
        self.label_11 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.label_11.setObjectName(u"label_11")
        self.label_11.setAlignment(Qt.AlignCenter)

        self.verticalLayout_11.addWidget(self.label_11)

        self.labelCarObsSensor9 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.labelCarObsSensor9.setObjectName(u"labelCarObsSensor9")

        self.verticalLayout_11.addWidget(self.labelCarObsSensor9)


        self.gridLayout_2.addLayout(self.verticalLayout_11, 3, 1, 1, 1)

        self.verticalLayout_6 = QVBoxLayout()
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.label_5 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setAlignment(Qt.AlignCenter)

        self.verticalLayout_6.addWidget(self.label_5)

        self.labelCarObsSensor11 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.labelCarObsSensor11.setObjectName(u"labelCarObsSensor11")

        self.verticalLayout_6.addWidget(self.labelCarObsSensor11)


        self.gridLayout_2.addLayout(self.verticalLayout_6, 0, 2, 1, 1)

        self.verticalLayout_5 = QVBoxLayout()
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.label_3 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setAlignment(Qt.AlignCenter)

        self.verticalLayout_5.addWidget(self.label_3)

        self.labelCarObsSensor10 = QLabel(self.groupBoxCarSensoresUltrasonicos)
        self.labelCarObsSensor10.setObjectName(u"labelCarObsSensor10")

        self.verticalLayout_5.addWidget(self.labelCarObsSensor10)


        self.gridLayout_2.addLayout(self.verticalLayout_5, 1, 2, 1, 1)


        self.horizontalLayout_5.addLayout(self.gridLayout_2)


        self.horizontalLayout_4.addWidget(self.groupBoxCarSensoresUltrasonicos)

        self.horizontalSpacer_7 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_4.addItem(self.horizontalSpacer_7)


        self.verticalLayout_15.addLayout(self.horizontalLayout_4)

        self.verticalSpacer_8 = QSpacerItem(20, 156, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_15.addItem(self.verticalSpacer_8)


        self.gridLayout.addWidget(self.groupBoxABBAutito, 1, 1, 2, 1)

        self.groupBoxABB2 = QGroupBox(self.groupBoxVisualizacion)
        self.groupBoxABB2.setObjectName(u"groupBoxABB2")
        sizePolicy2.setHeightForWidth(self.groupBoxABB2.sizePolicy().hasHeightForWidth())
        self.groupBoxABB2.setSizePolicy(sizePolicy2)
        self.horizontalLayout_2 = QHBoxLayout(self.groupBoxABB2)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalSpacer_4 = QSpacerItem(800, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_2.addItem(self.horizontalSpacer_4)

        self.widgetABB2 = QWidget(self.groupBoxABB2)
        self.widgetABB2.setObjectName(u"widgetABB2")
        sizePolicy3.setHeightForWidth(self.widgetABB2.sizePolicy().hasHeightForWidth())
        self.widgetABB2.setSizePolicy(sizePolicy3)
        self.widgetABB2.setMinimumSize(QSize(128, 128))
        self.widgetABB2.setMaximumSize(QSize(128, 128))
        self.widgetABB2.setStyleSheet(u"border-image: url(:/image/image/brazo.png) 0 0 0 0 stretch stretch;")

        self.horizontalLayout_2.addWidget(self.widgetABB2)

        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalSpacer_3 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_3.addItem(self.verticalSpacer_3)

        self.label_4 = QLabel(self.groupBoxABB2)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setAlignment(Qt.AlignCenter)

        self.verticalLayout_3.addWidget(self.label_4)

        self.labelABB2Accion = QLabel(self.groupBoxABB2)
        self.labelABB2Accion.setObjectName(u"labelABB2Accion")
        sizePolicy3.setHeightForWidth(self.labelABB2Accion.sizePolicy().hasHeightForWidth())
        self.labelABB2Accion.setSizePolicy(sizePolicy3)

        self.verticalLayout_3.addWidget(self.labelABB2Accion)

        self.label_6 = QLabel(self.groupBoxABB2)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setAlignment(Qt.AlignCenter)

        self.verticalLayout_3.addWidget(self.label_6)

        self.progressBarABB2 = QProgressBar(self.groupBoxABB2)
        self.progressBarABB2.setObjectName(u"progressBarABB2")
        self.progressBarABB2.setValue(0)

        self.verticalLayout_3.addWidget(self.progressBarABB2)

        self.verticalSpacer_4 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_3.addItem(self.verticalSpacer_4)


        self.horizontalLayout_2.addLayout(self.verticalLayout_3)

        self.widgetABB2Autito = QWidget(self.groupBoxABB2)
        self.widgetABB2Autito.setObjectName(u"widgetABB2Autito")
        sizePolicy3.setHeightForWidth(self.widgetABB2Autito.sizePolicy().hasHeightForWidth())
        self.widgetABB2Autito.setSizePolicy(sizePolicy3)
        self.widgetABB2Autito.setMinimumSize(QSize(128, 128))
        self.widgetABB2Autito.setMaximumSize(QSize(128, 128))
        self.widgetABB2Autito.setStyleSheet(u"border-image: url(:/image/image/car_no.png) 0 0 0 0 stretch stretch;")

        self.horizontalLayout_2.addWidget(self.widgetABB2Autito)

        self.horizontalSpacer_3 = QSpacerItem(800, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_2.addItem(self.horizontalSpacer_3)


        self.gridLayout.addWidget(self.groupBoxABB2, 1, 0, 1, 1)


        self.verticalLayout.addWidget(self.groupBoxVisualizacion)

        MainWindowTrabajoFinal.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindowTrabajoFinal)

        QMetaObject.connectSlotsByName(MainWindowTrabajoFinal)
    # setupUi

    def retranslateUi(self, MainWindowTrabajoFinal):
        MainWindowTrabajoFinal.setWindowTitle(QCoreApplication.translate("MainWindowTrabajoFinal", u"Escena final", None))
        self.groupBoxAcciones.setTitle(QCoreApplication.translate("MainWindowTrabajoFinal", u"Acciones", None))
        self.pushButtonIniciar.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"Iniciar", None))
        self.pushButtonDetener.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"Detener", None))
        self.groupBoxVisualizacion.setTitle(QCoreApplication.translate("MainWindowTrabajoFinal", u"Visualizaci\u00f3n", None))
        self.groupBoxABB1.setTitle(QCoreApplication.translate("MainWindowTrabajoFinal", u"ABB 1", None))
        self.label_2.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"Acci\u00f3n", None))
        self.labelABB1Accion.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"Esperando caja 1", None))
        self.label.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"Progreso", None))
        self.groupBoxABBAutito.setTitle(QCoreApplication.translate("MainWindowTrabajoFinal", u"Autito", None))
        self.groupBoxCarSensoresInfrarojos.setTitle(QCoreApplication.translate("MainWindowTrabajoFinal", u"Sensores infrarojos", None))
        self.label_15.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"S0", None))
        self.labelCarSensor0.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"0.0000", None))
        self.label_16.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"S1", None))
        self.labelCarSensor1.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"0.0000", None))
        self.label_17.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"S2", None))
        self.labelCarSensor2.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"0.0000", None))
        self.label_18.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"S3", None))
        self.labelCarSensor3.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"0.0000", None))
        self.label_19.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"S4", None))
        self.labelCarSensor4.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"0.0000", None))
        self.label_20.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"S5", None))
        self.labelCarSensor5.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"0.0000", None))
        self.groupBoxCarSensoresUltrasonicos.setTitle(QCoreApplication.translate("MainWindowTrabajoFinal", u"Sensores ultras\u00f3nicos", None))
        self.label_10.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"S15", None))
        self.labelCarObsSensor15.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"0.0000", None))
        self.label_13.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"S16", None))
        self.labelCarObsSensor16.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"0.0000", None))
        self.label_7.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"S12", None))
        self.labelCarObsSensor12.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"0.0000", None))
        self.label_14.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"S1", None))
        self.labelCarObsSensor1.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"0.0000", None))
        self.label_8.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"S13", None))
        self.labelCarObsSensor13.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"0.0000", None))
        self.label_12.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"S8", None))
        self.labelCarObsSensor8.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"0.0000", None))
        self.label_9.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"S14", None))
        self.labelCarObsSensor14.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"0.0000", None))
        self.label_11.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"S9", None))
        self.labelCarObsSensor9.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"0.0000", None))
        self.label_5.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"S11", None))
        self.labelCarObsSensor11.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"0.0000", None))
        self.label_3.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"S10", None))
        self.labelCarObsSensor10.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"0.0000", None))
        self.groupBoxABB2.setTitle(QCoreApplication.translate("MainWindowTrabajoFinal", u"ABB 2", None))
        self.label_4.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"Acci\u00f3n", None))
        self.labelABB2Accion.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"Esperando autito", None))
        self.label_6.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"Progreso", None))
    # retranslateUi

