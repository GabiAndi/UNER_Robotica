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
        sizePolicy1.setVerticalStretch(3)
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
        self.groupBoxABB2.setTitle(QCoreApplication.translate("MainWindowTrabajoFinal", u"ABB 2", None))
        self.label_4.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"Acci\u00f3n", None))
        self.labelABB2Accion.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"Esperando autito", None))
        self.label_6.setText(QCoreApplication.translate("MainWindowTrabajoFinal", u"Progreso", None))
    # retranslateUi

