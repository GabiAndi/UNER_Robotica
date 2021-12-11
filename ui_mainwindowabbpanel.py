# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'ui_mainwindowabbpanel.ui'
##
## Created by: Qt User Interface Compiler version 6.2.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QLineEdit, QMainWindow, QPushButton,
    QSizePolicy, QSlider, QSpacerItem, QVBoxLayout,
    QWidget)

class Ui_MainWindowABBPanel(object):
    def setupUi(self, MainWindowABBPanel):
        if not MainWindowABBPanel.objectName():
            MainWindowABBPanel.setObjectName(u"MainWindowABBPanel")
        MainWindowABBPanel.resize(731, 845)
        MainWindowABBPanel.setMinimumSize(QSize(600, 732))
        font = QFont()
        font.setFamilies([u"Nimbus Sans L"])
        font.setPointSize(11)
        MainWindowABBPanel.setFont(font)
        self.centralwidget = QWidget(MainWindowABBPanel)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout = QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.groupBoxCoppeliaSimulacion = QGroupBox(self.centralwidget)
        self.groupBoxCoppeliaSimulacion.setObjectName(u"groupBoxCoppeliaSimulacion")
        self.horizontalLayout = QHBoxLayout(self.groupBoxCoppeliaSimulacion)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.label = QLabel(self.groupBoxCoppeliaSimulacion)
        self.label.setObjectName(u"label")

        self.verticalLayout_2.addWidget(self.label)

        self.label_2 = QLabel(self.groupBoxCoppeliaSimulacion)
        self.label_2.setObjectName(u"label_2")

        self.verticalLayout_2.addWidget(self.label_2)


        self.horizontalLayout.addLayout(self.verticalLayout_2)

        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.lineEditCoppeliaIP = QLineEdit(self.groupBoxCoppeliaSimulacion)
        self.lineEditCoppeliaIP.setObjectName(u"lineEditCoppeliaIP")
        sizePolicy = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEditCoppeliaIP.sizePolicy().hasHeightForWidth())
        self.lineEditCoppeliaIP.setSizePolicy(sizePolicy)

        self.verticalLayout_3.addWidget(self.lineEditCoppeliaIP)

        self.lineEditCoppeliaPuerto = QLineEdit(self.groupBoxCoppeliaSimulacion)
        self.lineEditCoppeliaPuerto.setObjectName(u"lineEditCoppeliaPuerto")
        sizePolicy.setHeightForWidth(self.lineEditCoppeliaPuerto.sizePolicy().hasHeightForWidth())
        self.lineEditCoppeliaPuerto.setSizePolicy(sizePolicy)

        self.verticalLayout_3.addWidget(self.lineEditCoppeliaPuerto)


        self.horizontalLayout.addLayout(self.verticalLayout_3)

        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.pushButtonCoppeliaIniciarDetener = QPushButton(self.groupBoxCoppeliaSimulacion)
        self.pushButtonCoppeliaIniciarDetener.setObjectName(u"pushButtonCoppeliaIniciarDetener")

        self.verticalLayout_4.addWidget(self.pushButtonCoppeliaIniciarDetener)

        self.pushButtonCoppeliaConfiguracion = QPushButton(self.groupBoxCoppeliaSimulacion)
        self.pushButtonCoppeliaConfiguracion.setObjectName(u"pushButtonCoppeliaConfiguracion")

        self.verticalLayout_4.addWidget(self.pushButtonCoppeliaConfiguracion)


        self.horizontalLayout.addLayout(self.verticalLayout_4)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.label_3 = QLabel(self.groupBoxCoppeliaSimulacion)
        self.label_3.setObjectName(u"label_3")

        self.horizontalLayout_2.addWidget(self.label_3)

        self.labelCoppeliaEstadoConexion = QLabel(self.groupBoxCoppeliaSimulacion)
        self.labelCoppeliaEstadoConexion.setObjectName(u"labelCoppeliaEstadoConexion")

        self.horizontalLayout_2.addWidget(self.labelCoppeliaEstadoConexion)


        self.horizontalLayout.addLayout(self.horizontalLayout_2)

        self.horizontalSpacer = QSpacerItem(3, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)


        self.verticalLayout.addWidget(self.groupBoxCoppeliaSimulacion)

        self.horizontalLayout_9 = QHBoxLayout()
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.verticalLayout_7 = QVBoxLayout()
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.groupBoxABBPosicion = QGroupBox(self.centralwidget)
        self.groupBoxABBPosicion.setObjectName(u"groupBoxABBPosicion")
        self.groupBoxABBPosicion.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.verticalLayout_5 = QVBoxLayout(self.groupBoxABBPosicion)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.label_4 = QLabel(self.groupBoxABBPosicion)
        self.label_4.setObjectName(u"label_4")

        self.horizontalLayout_3.addWidget(self.label_4)

        self.labelABBPosicionX = QLabel(self.groupBoxABBPosicion)
        self.labelABBPosicionX.setObjectName(u"labelABBPosicionX")
        sizePolicy1 = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy1.setHorizontalStretch(1)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.labelABBPosicionX.sizePolicy().hasHeightForWidth())
        self.labelABBPosicionX.setSizePolicy(sizePolicy1)

        self.horizontalLayout_3.addWidget(self.labelABBPosicionX)


        self.verticalLayout_5.addLayout(self.horizontalLayout_3)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.label_5 = QLabel(self.groupBoxABBPosicion)
        self.label_5.setObjectName(u"label_5")

        self.horizontalLayout_4.addWidget(self.label_5)

        self.labelABBPosicionY = QLabel(self.groupBoxABBPosicion)
        self.labelABBPosicionY.setObjectName(u"labelABBPosicionY")
        sizePolicy1.setHeightForWidth(self.labelABBPosicionY.sizePolicy().hasHeightForWidth())
        self.labelABBPosicionY.setSizePolicy(sizePolicy1)

        self.horizontalLayout_4.addWidget(self.labelABBPosicionY)


        self.verticalLayout_5.addLayout(self.horizontalLayout_4)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.label_6 = QLabel(self.groupBoxABBPosicion)
        self.label_6.setObjectName(u"label_6")

        self.horizontalLayout_5.addWidget(self.label_6)

        self.labelABBPosicionZ = QLabel(self.groupBoxABBPosicion)
        self.labelABBPosicionZ.setObjectName(u"labelABBPosicionZ")
        sizePolicy1.setHeightForWidth(self.labelABBPosicionZ.sizePolicy().hasHeightForWidth())
        self.labelABBPosicionZ.setSizePolicy(sizePolicy1)

        self.horizontalLayout_5.addWidget(self.labelABBPosicionZ)


        self.verticalLayout_5.addLayout(self.horizontalLayout_5)


        self.verticalLayout_7.addWidget(self.groupBoxABBPosicion)

        self.groupBoxABBOrientacion = QGroupBox(self.centralwidget)
        self.groupBoxABBOrientacion.setObjectName(u"groupBoxABBOrientacion")
        self.groupBoxABBOrientacion.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.verticalLayout_6 = QVBoxLayout(self.groupBoxABBOrientacion)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.horizontalLayout_8 = QHBoxLayout()
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.label_9 = QLabel(self.groupBoxABBOrientacion)
        self.label_9.setObjectName(u"label_9")

        self.horizontalLayout_8.addWidget(self.label_9)

        self.labelABBOrientacionA = QLabel(self.groupBoxABBOrientacion)
        self.labelABBOrientacionA.setObjectName(u"labelABBOrientacionA")
        sizePolicy1.setHeightForWidth(self.labelABBOrientacionA.sizePolicy().hasHeightForWidth())
        self.labelABBOrientacionA.setSizePolicy(sizePolicy1)

        self.horizontalLayout_8.addWidget(self.labelABBOrientacionA)


        self.verticalLayout_6.addLayout(self.horizontalLayout_8)

        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.label_8 = QLabel(self.groupBoxABBOrientacion)
        self.label_8.setObjectName(u"label_8")

        self.horizontalLayout_7.addWidget(self.label_8)

        self.labelABBOrientacionB = QLabel(self.groupBoxABBOrientacion)
        self.labelABBOrientacionB.setObjectName(u"labelABBOrientacionB")
        sizePolicy1.setHeightForWidth(self.labelABBOrientacionB.sizePolicy().hasHeightForWidth())
        self.labelABBOrientacionB.setSizePolicy(sizePolicy1)

        self.horizontalLayout_7.addWidget(self.labelABBOrientacionB)


        self.verticalLayout_6.addLayout(self.horizontalLayout_7)

        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.label_7 = QLabel(self.groupBoxABBOrientacion)
        self.label_7.setObjectName(u"label_7")

        self.horizontalLayout_6.addWidget(self.label_7)

        self.labelABBOrientacionC = QLabel(self.groupBoxABBOrientacion)
        self.labelABBOrientacionC.setObjectName(u"labelABBOrientacionC")
        sizePolicy1.setHeightForWidth(self.labelABBOrientacionC.sizePolicy().hasHeightForWidth())
        self.labelABBOrientacionC.setSizePolicy(sizePolicy1)

        self.horizontalLayout_6.addWidget(self.labelABBOrientacionC)


        self.verticalLayout_6.addLayout(self.horizontalLayout_6)


        self.verticalLayout_7.addWidget(self.groupBoxABBOrientacion)

        self.groupBoxABBPasos = QGroupBox(self.centralwidget)
        self.groupBoxABBPasos.setObjectName(u"groupBoxABBPasos")
        self.groupBoxABBPasos.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.horizontalLayout_10 = QHBoxLayout(self.groupBoxABBPasos)
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.horizontalSliderABBPasos = QSlider(self.groupBoxABBPasos)
        self.horizontalSliderABBPasos.setObjectName(u"horizontalSliderABBPasos")
        self.horizontalSliderABBPasos.setMinimum(1)
        self.horizontalSliderABBPasos.setMaximum(100)
        self.horizontalSliderABBPasos.setSingleStep(1)
        self.horizontalSliderABBPasos.setValue(10)
        self.horizontalSliderABBPasos.setSliderPosition(10)
        self.horizontalSliderABBPasos.setOrientation(Qt.Horizontal)

        self.horizontalLayout_10.addWidget(self.horizontalSliderABBPasos)

        self.labelABBPasos = QLabel(self.groupBoxABBPasos)
        self.labelABBPasos.setObjectName(u"labelABBPasos")

        self.horizontalLayout_10.addWidget(self.labelABBPasos)


        self.verticalLayout_7.addWidget(self.groupBoxABBPasos)

        self.groupBoxABBTrayectoria = QGroupBox(self.centralwidget)
        self.groupBoxABBTrayectoria.setObjectName(u"groupBoxABBTrayectoria")
        self.groupBoxABBTrayectoria.setEnabled(False)
        self.groupBoxABBTrayectoria.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.verticalLayout_10 = QVBoxLayout(self.groupBoxABBTrayectoria)
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.pushButtonABBCapturarPunto = QPushButton(self.groupBoxABBTrayectoria)
        self.pushButtonABBCapturarPunto.setObjectName(u"pushButtonABBCapturarPunto")

        self.verticalLayout_10.addWidget(self.pushButtonABBCapturarPunto)

        self.pushButtonABBBorrarUltimoPunto = QPushButton(self.groupBoxABBTrayectoria)
        self.pushButtonABBBorrarUltimoPunto.setObjectName(u"pushButtonABBBorrarUltimoPunto")

        self.verticalLayout_10.addWidget(self.pushButtonABBBorrarUltimoPunto)

        self.pushButtonABBLimpiarTrayectoria = QPushButton(self.groupBoxABBTrayectoria)
        self.pushButtonABBLimpiarTrayectoria.setObjectName(u"pushButtonABBLimpiarTrayectoria")

        self.verticalLayout_10.addWidget(self.pushButtonABBLimpiarTrayectoria)

        self.horizontalLayout_19 = QHBoxLayout()
        self.horizontalLayout_19.setObjectName(u"horizontalLayout_19")
        self.label_12 = QLabel(self.groupBoxABBTrayectoria)
        self.label_12.setObjectName(u"label_12")

        self.horizontalLayout_19.addWidget(self.label_12)

        self.labelABBCantidadPuntos = QLabel(self.groupBoxABBTrayectoria)
        self.labelABBCantidadPuntos.setObjectName(u"labelABBCantidadPuntos")
        sizePolicy1.setHeightForWidth(self.labelABBCantidadPuntos.sizePolicy().hasHeightForWidth())
        self.labelABBCantidadPuntos.setSizePolicy(sizePolicy1)

        self.horizontalLayout_19.addWidget(self.labelABBCantidadPuntos)


        self.verticalLayout_10.addLayout(self.horizontalLayout_19)

        self.pushButtonABBAnteriorTrayectoria = QPushButton(self.groupBoxABBTrayectoria)
        self.pushButtonABBAnteriorTrayectoria.setObjectName(u"pushButtonABBAnteriorTrayectoria")

        self.verticalLayout_10.addWidget(self.pushButtonABBAnteriorTrayectoria)

        self.pushButtonABBSiguienteTrayectoria = QPushButton(self.groupBoxABBTrayectoria)
        self.pushButtonABBSiguienteTrayectoria.setObjectName(u"pushButtonABBSiguienteTrayectoria")

        self.verticalLayout_10.addWidget(self.pushButtonABBSiguienteTrayectoria)

        self.pushButtonABBEjecutarPuntos = QPushButton(self.groupBoxABBTrayectoria)
        self.pushButtonABBEjecutarPuntos.setObjectName(u"pushButtonABBEjecutarPuntos")

        self.verticalLayout_10.addWidget(self.pushButtonABBEjecutarPuntos)

        self.pushButtonABBGuardarTrayectoria = QPushButton(self.groupBoxABBTrayectoria)
        self.pushButtonABBGuardarTrayectoria.setObjectName(u"pushButtonABBGuardarTrayectoria")

        self.verticalLayout_10.addWidget(self.pushButtonABBGuardarTrayectoria)

        self.pushButtonABBCargarTrayectoria = QPushButton(self.groupBoxABBTrayectoria)
        self.pushButtonABBCargarTrayectoria.setObjectName(u"pushButtonABBCargarTrayectoria")

        self.verticalLayout_10.addWidget(self.pushButtonABBCargarTrayectoria)


        self.verticalLayout_7.addWidget(self.groupBoxABBTrayectoria)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_7.addItem(self.verticalSpacer)


        self.horizontalLayout_9.addLayout(self.verticalLayout_7)

        self.groupBoxABBControl = QGroupBox(self.centralwidget)
        self.groupBoxABBControl.setObjectName(u"groupBoxABBControl")
        self.groupBoxABBControl.setEnabled(False)
        sizePolicy2 = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy2.setHorizontalStretch(1)
        sizePolicy2.setVerticalStretch(1)
        sizePolicy2.setHeightForWidth(self.groupBoxABBControl.sizePolicy().hasHeightForWidth())
        self.groupBoxABBControl.setSizePolicy(sizePolicy2)
        self.groupBoxABBControl.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)
        self.horizontalLayout_17 = QHBoxLayout(self.groupBoxABBControl)
        self.horizontalLayout_17.setObjectName(u"horizontalLayout_17")
        self.horizontalSpacer_3 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_17.addItem(self.horizontalSpacer_3)

        self.verticalLayout_11 = QVBoxLayout()
        self.verticalLayout_11.setObjectName(u"verticalLayout_11")
        self.verticalSpacer_4 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_11.addItem(self.verticalSpacer_4)

        self.verticalLayout_8 = QVBoxLayout()
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.label_10 = QLabel(self.groupBoxABBControl)
        self.label_10.setObjectName(u"label_10")
        self.label_10.setAlignment(Qt.AlignCenter)

        self.verticalLayout_8.addWidget(self.label_10)

        self.gridLayout_2 = QGridLayout()
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.pushButtonABBYUp = QPushButton(self.groupBoxABBControl)
        self.pushButtonABBYUp.setObjectName(u"pushButtonABBYUp")

        self.gridLayout_2.addWidget(self.pushButtonABBYUp, 0, 2, 1, 1)

        self.pushButtonABBXDown = QPushButton(self.groupBoxABBControl)
        self.pushButtonABBXDown.setObjectName(u"pushButtonABBXDown")

        self.gridLayout_2.addWidget(self.pushButtonABBXDown, 1, 1, 1, 1)

        self.pushButtonABBXUp = QPushButton(self.groupBoxABBControl)
        self.pushButtonABBXUp.setObjectName(u"pushButtonABBXUp")

        self.gridLayout_2.addWidget(self.pushButtonABBXUp, 1, 3, 1, 1)

        self.pushButtonABBYDown = QPushButton(self.groupBoxABBControl)
        self.pushButtonABBYDown.setObjectName(u"pushButtonABBYDown")

        self.gridLayout_2.addWidget(self.pushButtonABBYDown, 1, 2, 1, 1)

        self.pushButtonABBZUp = QPushButton(self.groupBoxABBControl)
        self.pushButtonABBZUp.setObjectName(u"pushButtonABBZUp")

        self.gridLayout_2.addWidget(self.pushButtonABBZUp, 0, 3, 1, 1)

        self.pushButtonABBZDown = QPushButton(self.groupBoxABBControl)
        self.pushButtonABBZDown.setObjectName(u"pushButtonABBZDown")

        self.gridLayout_2.addWidget(self.pushButtonABBZDown, 0, 1, 1, 1)


        self.verticalLayout_8.addLayout(self.gridLayout_2)


        self.verticalLayout_11.addLayout(self.verticalLayout_8)

        self.verticalLayout_9 = QVBoxLayout()
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.label_11 = QLabel(self.groupBoxABBControl)
        self.label_11.setObjectName(u"label_11")
        self.label_11.setAlignment(Qt.AlignCenter)

        self.verticalLayout_9.addWidget(self.label_11)

        self.gridLayout_3 = QGridLayout()
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.pushButtonABBADown = QPushButton(self.groupBoxABBControl)
        self.pushButtonABBADown.setObjectName(u"pushButtonABBADown")

        self.gridLayout_3.addWidget(self.pushButtonABBADown, 1, 1, 1, 1)

        self.pushButtonABBAUp = QPushButton(self.groupBoxABBControl)
        self.pushButtonABBAUp.setObjectName(u"pushButtonABBAUp")

        self.gridLayout_3.addWidget(self.pushButtonABBAUp, 1, 3, 1, 1)

        self.pushButtonABBBUp = QPushButton(self.groupBoxABBControl)
        self.pushButtonABBBUp.setObjectName(u"pushButtonABBBUp")

        self.gridLayout_3.addWidget(self.pushButtonABBBUp, 0, 2, 1, 1)

        self.pushButtonABBBDown = QPushButton(self.groupBoxABBControl)
        self.pushButtonABBBDown.setObjectName(u"pushButtonABBBDown")

        self.gridLayout_3.addWidget(self.pushButtonABBBDown, 2, 2, 1, 1)

        self.pushButtonABBCDown = QPushButton(self.groupBoxABBControl)
        self.pushButtonABBCDown.setObjectName(u"pushButtonABBCDown")

        self.gridLayout_3.addWidget(self.pushButtonABBCDown, 2, 1, 1, 1)

        self.pushButtonABBCUp = QPushButton(self.groupBoxABBControl)
        self.pushButtonABBCUp.setObjectName(u"pushButtonABBCUp")

        self.gridLayout_3.addWidget(self.pushButtonABBCUp, 0, 1, 1, 1)


        self.verticalLayout_9.addLayout(self.gridLayout_3)


        self.verticalLayout_11.addLayout(self.verticalLayout_9)

        self.verticalSpacer_3 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_11.addItem(self.verticalSpacer_3)


        self.horizontalLayout_17.addLayout(self.verticalLayout_11)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_17.addItem(self.horizontalSpacer_2)


        self.horizontalLayout_9.addWidget(self.groupBoxABBControl)

        self.verticalLayout_14 = QVBoxLayout()
        self.verticalLayout_14.setObjectName(u"verticalLayout_14")
        self.groupBoxABBJuntas = QGroupBox(self.centralwidget)
        self.groupBoxABBJuntas.setObjectName(u"groupBoxABBJuntas")
        self.verticalLayout_13 = QVBoxLayout(self.groupBoxABBJuntas)
        self.verticalLayout_13.setObjectName(u"verticalLayout_13")
        self.horizontalLayout_11 = QHBoxLayout()
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.label_19 = QLabel(self.groupBoxABBJuntas)
        self.label_19.setObjectName(u"label_19")

        self.horizontalLayout_11.addWidget(self.label_19)

        self.labelABBQ1 = QLabel(self.groupBoxABBJuntas)
        self.labelABBQ1.setObjectName(u"labelABBQ1")
        sizePolicy1.setHeightForWidth(self.labelABBQ1.sizePolicy().hasHeightForWidth())
        self.labelABBQ1.setSizePolicy(sizePolicy1)

        self.horizontalLayout_11.addWidget(self.labelABBQ1)


        self.verticalLayout_13.addLayout(self.horizontalLayout_11)

        self.horizontalLayout_12 = QHBoxLayout()
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.label_21 = QLabel(self.groupBoxABBJuntas)
        self.label_21.setObjectName(u"label_21")

        self.horizontalLayout_12.addWidget(self.label_21)

        self.labelABBQ2 = QLabel(self.groupBoxABBJuntas)
        self.labelABBQ2.setObjectName(u"labelABBQ2")
        sizePolicy1.setHeightForWidth(self.labelABBQ2.sizePolicy().hasHeightForWidth())
        self.labelABBQ2.setSizePolicy(sizePolicy1)

        self.horizontalLayout_12.addWidget(self.labelABBQ2)


        self.verticalLayout_13.addLayout(self.horizontalLayout_12)

        self.horizontalLayout_13 = QHBoxLayout()
        self.horizontalLayout_13.setObjectName(u"horizontalLayout_13")
        self.label_22 = QLabel(self.groupBoxABBJuntas)
        self.label_22.setObjectName(u"label_22")

        self.horizontalLayout_13.addWidget(self.label_22)

        self.labelABBQ3 = QLabel(self.groupBoxABBJuntas)
        self.labelABBQ3.setObjectName(u"labelABBQ3")
        sizePolicy1.setHeightForWidth(self.labelABBQ3.sizePolicy().hasHeightForWidth())
        self.labelABBQ3.setSizePolicy(sizePolicy1)

        self.horizontalLayout_13.addWidget(self.labelABBQ3)


        self.verticalLayout_13.addLayout(self.horizontalLayout_13)

        self.horizontalLayout_15 = QHBoxLayout()
        self.horizontalLayout_15.setObjectName(u"horizontalLayout_15")
        self.label_23 = QLabel(self.groupBoxABBJuntas)
        self.label_23.setObjectName(u"label_23")

        self.horizontalLayout_15.addWidget(self.label_23)

        self.labelABBQ4 = QLabel(self.groupBoxABBJuntas)
        self.labelABBQ4.setObjectName(u"labelABBQ4")
        sizePolicy1.setHeightForWidth(self.labelABBQ4.sizePolicy().hasHeightForWidth())
        self.labelABBQ4.setSizePolicy(sizePolicy1)

        self.horizontalLayout_15.addWidget(self.labelABBQ4)


        self.verticalLayout_13.addLayout(self.horizontalLayout_15)

        self.horizontalLayout_14 = QHBoxLayout()
        self.horizontalLayout_14.setObjectName(u"horizontalLayout_14")
        self.label_20 = QLabel(self.groupBoxABBJuntas)
        self.label_20.setObjectName(u"label_20")

        self.horizontalLayout_14.addWidget(self.label_20)

        self.labelABBQ5 = QLabel(self.groupBoxABBJuntas)
        self.labelABBQ5.setObjectName(u"labelABBQ5")
        sizePolicy1.setHeightForWidth(self.labelABBQ5.sizePolicy().hasHeightForWidth())
        self.labelABBQ5.setSizePolicy(sizePolicy1)

        self.horizontalLayout_14.addWidget(self.labelABBQ5)


        self.verticalLayout_13.addLayout(self.horizontalLayout_14)

        self.horizontalLayout_16 = QHBoxLayout()
        self.horizontalLayout_16.setObjectName(u"horizontalLayout_16")
        self.label_24 = QLabel(self.groupBoxABBJuntas)
        self.label_24.setObjectName(u"label_24")

        self.horizontalLayout_16.addWidget(self.label_24)

        self.labelABBQ6 = QLabel(self.groupBoxABBJuntas)
        self.labelABBQ6.setObjectName(u"labelABBQ6")
        sizePolicy1.setHeightForWidth(self.labelABBQ6.sizePolicy().hasHeightForWidth())
        self.labelABBQ6.setSizePolicy(sizePolicy1)

        self.horizontalLayout_16.addWidget(self.labelABBQ6)


        self.verticalLayout_13.addLayout(self.horizontalLayout_16)


        self.verticalLayout_14.addWidget(self.groupBoxABBJuntas)

        self.groupBoxABBHerramienta = QGroupBox(self.centralwidget)
        self.groupBoxABBHerramienta.setObjectName(u"groupBoxABBHerramienta")
        self.groupBoxABBHerramienta.setEnabled(False)
        self.verticalLayout_12 = QVBoxLayout(self.groupBoxABBHerramienta)
        self.verticalLayout_12.setObjectName(u"verticalLayout_12")
        self.pushButtonABBActivarHerramienta = QPushButton(self.groupBoxABBHerramienta)
        self.pushButtonABBActivarHerramienta.setObjectName(u"pushButtonABBActivarHerramienta")

        self.verticalLayout_12.addWidget(self.pushButtonABBActivarHerramienta)

        self.pushButtonABBDesactivarHerramienta = QPushButton(self.groupBoxABBHerramienta)
        self.pushButtonABBDesactivarHerramienta.setObjectName(u"pushButtonABBDesactivarHerramienta")

        self.verticalLayout_12.addWidget(self.pushButtonABBDesactivarHerramienta)


        self.verticalLayout_14.addWidget(self.groupBoxABBHerramienta)

        self.verticalSpacer_6 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_14.addItem(self.verticalSpacer_6)


        self.horizontalLayout_9.addLayout(self.verticalLayout_14)


        self.verticalLayout.addLayout(self.horizontalLayout_9)

        MainWindowABBPanel.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindowABBPanel)
        self.horizontalSliderABBPasos.valueChanged.connect(self.labelABBPasos.setNum)

        QMetaObject.connectSlotsByName(MainWindowABBPanel)
    # setupUi

    def retranslateUi(self, MainWindowABBPanel):
        MainWindowABBPanel.setWindowTitle(QCoreApplication.translate("MainWindowABBPanel", u"ABB panel", None))
        self.groupBoxCoppeliaSimulacion.setTitle(QCoreApplication.translate("MainWindowABBPanel", u"Simulaci\u00f3n", None))
        self.label.setText(QCoreApplication.translate("MainWindowABBPanel", u"Direcci\u00f3n:", None))
        self.label_2.setText(QCoreApplication.translate("MainWindowABBPanel", u"Puerto:", None))
        self.lineEditCoppeliaIP.setText(QCoreApplication.translate("MainWindowABBPanel", u"127.0.0.1", None))
        self.lineEditCoppeliaPuerto.setText(QCoreApplication.translate("MainWindowABBPanel", u"19999", None))
        self.pushButtonCoppeliaIniciarDetener.setText(QCoreApplication.translate("MainWindowABBPanel", u"Iniciar", None))
        self.pushButtonCoppeliaConfiguracion.setText(QCoreApplication.translate("MainWindowABBPanel", u"Configuraci\u00f3n", None))
        self.label_3.setText(QCoreApplication.translate("MainWindowABBPanel", u"Estado:", None))
        self.labelCoppeliaEstadoConexion.setText(QCoreApplication.translate("MainWindowABBPanel", u"desconectado", None))
        self.groupBoxABBPosicion.setTitle(QCoreApplication.translate("MainWindowABBPanel", u"Posici\u00f3n", None))
        self.label_4.setText(QCoreApplication.translate("MainWindowABBPanel", u"X:", None))
        self.labelABBPosicionX.setText(QCoreApplication.translate("MainWindowABBPanel", u"374.00", None))
        self.label_5.setText(QCoreApplication.translate("MainWindowABBPanel", u"Y:", None))
        self.labelABBPosicionY.setText(QCoreApplication.translate("MainWindowABBPanel", u"0.00", None))
        self.label_6.setText(QCoreApplication.translate("MainWindowABBPanel", u"Z:", None))
        self.labelABBPosicionZ.setText(QCoreApplication.translate("MainWindowABBPanel", u"630.00", None))
        self.groupBoxABBOrientacion.setTitle(QCoreApplication.translate("MainWindowABBPanel", u"Orientaci\u00f3n", None))
        self.label_9.setText(QCoreApplication.translate("MainWindowABBPanel", u"A:", None))
        self.labelABBOrientacionA.setText(QCoreApplication.translate("MainWindowABBPanel", u"180.00", None))
        self.label_8.setText(QCoreApplication.translate("MainWindowABBPanel", u"B:", None))
        self.labelABBOrientacionB.setText(QCoreApplication.translate("MainWindowABBPanel", u"90.00", None))
        self.label_7.setText(QCoreApplication.translate("MainWindowABBPanel", u"C:", None))
        self.labelABBOrientacionC.setText(QCoreApplication.translate("MainWindowABBPanel", u"0.00", None))
        self.groupBoxABBPasos.setTitle(QCoreApplication.translate("MainWindowABBPanel", u"Pasos", None))
        self.labelABBPasos.setText(QCoreApplication.translate("MainWindowABBPanel", u"10", None))
        self.groupBoxABBTrayectoria.setTitle(QCoreApplication.translate("MainWindowABBPanel", u"Trayectoria", None))
        self.pushButtonABBCapturarPunto.setText(QCoreApplication.translate("MainWindowABBPanel", u"Capturar", None))
        self.pushButtonABBBorrarUltimoPunto.setText(QCoreApplication.translate("MainWindowABBPanel", u"Borrar \u00faltimo", None))
        self.pushButtonABBLimpiarTrayectoria.setText(QCoreApplication.translate("MainWindowABBPanel", u"Limpiar", None))
        self.label_12.setText(QCoreApplication.translate("MainWindowABBPanel", u"Puntos:", None))
        self.labelABBCantidadPuntos.setText(QCoreApplication.translate("MainWindowABBPanel", u"0", None))
        self.pushButtonABBAnteriorTrayectoria.setText(QCoreApplication.translate("MainWindowABBPanel", u"Anterior", None))
        self.pushButtonABBSiguienteTrayectoria.setText(QCoreApplication.translate("MainWindowABBPanel", u"Siguiente", None))
        self.pushButtonABBEjecutarPuntos.setText(QCoreApplication.translate("MainWindowABBPanel", u"Ejecutar", None))
        self.pushButtonABBGuardarTrayectoria.setText(QCoreApplication.translate("MainWindowABBPanel", u"Guardar", None))
        self.pushButtonABBCargarTrayectoria.setText(QCoreApplication.translate("MainWindowABBPanel", u"Cargar", None))
        self.groupBoxABBControl.setTitle(QCoreApplication.translate("MainWindowABBPanel", u"Control", None))
        self.label_10.setText(QCoreApplication.translate("MainWindowABBPanel", u"Posici\u00f3n", None))
        self.pushButtonABBYUp.setText(QCoreApplication.translate("MainWindowABBPanel", u"Y+", None))
#if QT_CONFIG(shortcut)
        self.pushButtonABBYUp.setShortcut(QCoreApplication.translate("MainWindowABBPanel", u"Ctrl+W", None))
#endif // QT_CONFIG(shortcut)
        self.pushButtonABBXDown.setText(QCoreApplication.translate("MainWindowABBPanel", u"X-", None))
#if QT_CONFIG(shortcut)
        self.pushButtonABBXDown.setShortcut(QCoreApplication.translate("MainWindowABBPanel", u"Ctrl+A", None))
#endif // QT_CONFIG(shortcut)
        self.pushButtonABBXUp.setText(QCoreApplication.translate("MainWindowABBPanel", u"X+", None))
#if QT_CONFIG(shortcut)
        self.pushButtonABBXUp.setShortcut(QCoreApplication.translate("MainWindowABBPanel", u"Ctrl+D", None))
#endif // QT_CONFIG(shortcut)
        self.pushButtonABBYDown.setText(QCoreApplication.translate("MainWindowABBPanel", u"Y-", None))
#if QT_CONFIG(shortcut)
        self.pushButtonABBYDown.setShortcut(QCoreApplication.translate("MainWindowABBPanel", u"Ctrl+S", None))
#endif // QT_CONFIG(shortcut)
        self.pushButtonABBZUp.setText(QCoreApplication.translate("MainWindowABBPanel", u"Z+", None))
#if QT_CONFIG(shortcut)
        self.pushButtonABBZUp.setShortcut(QCoreApplication.translate("MainWindowABBPanel", u"Ctrl+E", None))
#endif // QT_CONFIG(shortcut)
        self.pushButtonABBZDown.setText(QCoreApplication.translate("MainWindowABBPanel", u"Z-", None))
#if QT_CONFIG(shortcut)
        self.pushButtonABBZDown.setShortcut(QCoreApplication.translate("MainWindowABBPanel", u"Ctrl+Q", None))
#endif // QT_CONFIG(shortcut)
        self.label_11.setText(QCoreApplication.translate("MainWindowABBPanel", u"Orientaci\u00f3n", None))
        self.pushButtonABBADown.setText(QCoreApplication.translate("MainWindowABBPanel", u"A-", None))
#if QT_CONFIG(shortcut)
        self.pushButtonABBADown.setShortcut(QCoreApplication.translate("MainWindowABBPanel", u"Ctrl+4", None))
#endif // QT_CONFIG(shortcut)
        self.pushButtonABBAUp.setText(QCoreApplication.translate("MainWindowABBPanel", u"A+", None))
#if QT_CONFIG(shortcut)
        self.pushButtonABBAUp.setShortcut(QCoreApplication.translate("MainWindowABBPanel", u"Ctrl+6", None))
#endif // QT_CONFIG(shortcut)
        self.pushButtonABBBUp.setText(QCoreApplication.translate("MainWindowABBPanel", u"B+", None))
#if QT_CONFIG(shortcut)
        self.pushButtonABBBUp.setShortcut(QCoreApplication.translate("MainWindowABBPanel", u"Ctrl+8", None))
#endif // QT_CONFIG(shortcut)
        self.pushButtonABBBDown.setText(QCoreApplication.translate("MainWindowABBPanel", u"B-", None))
#if QT_CONFIG(shortcut)
        self.pushButtonABBBDown.setShortcut(QCoreApplication.translate("MainWindowABBPanel", u"Ctrl+2", None))
#endif // QT_CONFIG(shortcut)
        self.pushButtonABBCDown.setText(QCoreApplication.translate("MainWindowABBPanel", u"C-", None))
#if QT_CONFIG(shortcut)
        self.pushButtonABBCDown.setShortcut(QCoreApplication.translate("MainWindowABBPanel", u"Ctrl+1", None))
#endif // QT_CONFIG(shortcut)
        self.pushButtonABBCUp.setText(QCoreApplication.translate("MainWindowABBPanel", u"C+", None))
#if QT_CONFIG(shortcut)
        self.pushButtonABBCUp.setShortcut(QCoreApplication.translate("MainWindowABBPanel", u"Ctrl+7", None))
#endif // QT_CONFIG(shortcut)
        self.groupBoxABBJuntas.setTitle(QCoreApplication.translate("MainWindowABBPanel", u"Juntas", None))
        self.label_19.setText(QCoreApplication.translate("MainWindowABBPanel", u"Q1", None))
        self.labelABBQ1.setText(QCoreApplication.translate("MainWindowABBPanel", u"0.00", None))
        self.label_21.setText(QCoreApplication.translate("MainWindowABBPanel", u"Q2", None))
        self.labelABBQ2.setText(QCoreApplication.translate("MainWindowABBPanel", u"0.00", None))
        self.label_22.setText(QCoreApplication.translate("MainWindowABBPanel", u"Q3", None))
        self.labelABBQ3.setText(QCoreApplication.translate("MainWindowABBPanel", u"0.00", None))
        self.label_23.setText(QCoreApplication.translate("MainWindowABBPanel", u"Q4", None))
        self.labelABBQ4.setText(QCoreApplication.translate("MainWindowABBPanel", u"0.00", None))
        self.label_20.setText(QCoreApplication.translate("MainWindowABBPanel", u"Q5", None))
        self.labelABBQ5.setText(QCoreApplication.translate("MainWindowABBPanel", u"0.00", None))
        self.label_24.setText(QCoreApplication.translate("MainWindowABBPanel", u"Q6", None))
        self.labelABBQ6.setText(QCoreApplication.translate("MainWindowABBPanel", u"0.00", None))
        self.groupBoxABBHerramienta.setTitle(QCoreApplication.translate("MainWindowABBPanel", u"Herramienta", None))
        self.pushButtonABBActivarHerramienta.setText(QCoreApplication.translate("MainWindowABBPanel", u"Activar", None))
        self.pushButtonABBDesactivarHerramienta.setText(QCoreApplication.translate("MainWindowABBPanel", u"Desactivar", None))
    # retranslateUi

