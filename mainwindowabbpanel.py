from PySide2.QtWidgets import QMainWindow, QMessageBox, QFileDialog
from PySide2.QtCore import Signal

from ui_mainwindowabbpanel import Ui_MainWindowABBPanel

import numpy as np
import sim
import time
import os

from abbengine import ABBEngine


class MainWindowABBPanel(QMainWindow):
    # Eventos
    closed = Signal()

    def __init__(self, parent=None):
        super(MainWindowABBPanel, self).__init__(parent)

        self.ui = Ui_MainWindowABBPanel()
        self.ui.setupUi(self)

        # Motor de cinematica
        self.abbengine = ABBEngine()

        # Posiciones de interes
        self.p_home = [374.0, 0.0, 630.0, np.pi, np.pi / 2.0, 0.0]

        # Valores actuales del robot
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.q4 = 0.0
        self.q5 = 0.0
        self.q6 = 0.0

        self.x = self.p_home[0]
        self.y = self.p_home[1]
        self.z = self.p_home[2]

        self.a = self.p_home[3]
        self.b = self.p_home[4]
        self.c = self.p_home[5]

        # Valores iniciales
        self.coppeliaIdClient = -1

        # Juntas
        self.coppeliaJoint1Name = "IRB120_joint_1"
        self.coppeliaJoint2Name = "IRB120_joint_2"
        self.coppeliaJoint3Name = "IRB120_joint_3"
        self.coppeliaJoint4Name = "IRB120_joint_4"
        self.coppeliaJoint5Name = "IRB120_joint_5"
        self.coppeliaJoint6Name = "IRB120_joint_6"

        self.coppeliaJoint1Handle = None
        self.coppeliaJoint2Handle = None
        self.coppeliaJoint3Handle = None
        self.coppeliaJoint4Handle = None
        self.coppeliaJoint5Handle = None
        self.coppeliaJoint6Handle = None

        # Vector de trayectorias
        self.trajectory = []

        # Eventos de botones
        self.ui.pushButtonCoppeliaIniciarDetener.clicked.connect(self.pushButton_coppeliaIniciarDetener_onClicked)

        self.ui.pushButtonABBXDown.clicked.connect(self.pushButton_pushButtonABBXDown_onClicked)
        self.ui.pushButtonABBXUp.clicked.connect(self.pushButton_pushButtonABBXUp_onClicked)
        self.ui.pushButtonABBYDown.clicked.connect(self.pushButton_pushButtonABBYDown_onClicked)
        self.ui.pushButtonABBYUp.clicked.connect(self.pushButton_pushButtonABBYUp_onClicked)
        self.ui.pushButtonABBZDown.clicked.connect(self.pushButton_pushButtonABBZDown_onClicked)
        self.ui.pushButtonABBZUp.clicked.connect(self.pushButton_pushButtonABBZUp_onClicked)

        self.ui.pushButtonABBADown.clicked.connect(self.pushButton_pushButtonABBADown_onClicked)
        self.ui.pushButtonABBAUp.clicked.connect(self.pushButton_pushButtonABBAUp_onClicked)
        self.ui.pushButtonABBBDown.clicked.connect(self.pushButton_pushButtonABBBDown_onClicked)
        self.ui.pushButtonABBBUp.clicked.connect(self.pushButton_pushButtonABBBUp_onClicked)
        self.ui.pushButtonABBCDown.clicked.connect(self.pushButton_pushButtonABBCDown_onClicked)
        self.ui.pushButtonABBCUp.clicked.connect(self.pushButton_pushButtonABBCUp_onClicked)

        self.ui.pushButtonABBCapturarPunto.clicked.connect(self.pushButton_pushButtonABBCapturarPunto_onClicked)
        self.ui.pushButtonABBLimpiarTrayectoria.clicked.connect(
            self.pushButton_pushButtonABBLimpiarTrayectoria_onClicked
        )
        self.ui.pushButtonABBEjecutarPuntos.clicked.connect(self.pushButton_pushButtonABBEjecutarTrayectoria_onClicked)
        self.ui.pushButtonABBGuardarTrayectoria.clicked.connect(self.pushButton_pushButtonABBGuardarTrayectoria_onClicked)

    def closeEvent(self, event):
        self.coppeliaDisconnect()

        self.closed.emit()

    def pushButton_coppeliaIniciarDetener_onClicked(self):
        if not self.coppeliaIsConnected():
            if self.coppeliaConnect(self.ui.lineEditCoppeliaIP.text(), int(self.ui.lineEditCoppeliaPuerto.text())):
                if self.coppeliaBegin():
                    QMessageBox(QMessageBox.Information, "Simulación", "Se conecto a la simulación correctamente",
                                QMessageBox.Ok, self).open()

                    self.ui.labelCoppeliaEstadoConexion.setText("conectado")
                    self.ui.pushButtonCoppeliaIniciarDetener.setText("Detener")
                    self.ui.pushButtonCoppeliaConfiguracion.setEnabled(False)
                    self.ui.groupBoxABBControl.setEnabled(True)
                    self.ui.groupBoxABBTrayectoria.setEnabled(True)

                else:
                    QMessageBox(QMessageBox.Critical, "Simulación", "Error en la configuración",
                                QMessageBox.Ok, self).open()

                    self.coppeliaDisconnect()

            else:
                QMessageBox(QMessageBox.Critical, "Simulación", "No se pudo conectar con la simulacion",
                            QMessageBox.Ok, self).open()

        else:
            self.coppeliaDisconnect()

            QMessageBox(QMessageBox.Warning, "Simulación", "Desconectado de la simulacion",
                        QMessageBox.Ok, self).open()

            self.ui.labelCoppeliaEstadoConexion.setText("deconectado")
            self.ui.pushButtonCoppeliaIniciarDetener.setText("Iniciar")
            self.ui.pushButtonCoppeliaConfiguracion.setEnabled(True)
            self.ui.groupBoxABBControl.setEnabled(False)
            self.ui.groupBoxABBTrayectoria.setEnabled(False)

    def pushButton_pushButtonABBXDown_onClicked(self):
        _x = self.x - self.ui.horizontalSliderABBPasos.value()

        sol = self.abbengine.inverseKinematic(_x, self.y, self.z, self.a, self.b, self.c)

        if sol is not None:
            self.x = _x

            [self.q1, self.q2, self.q3, self.q4, self.q5, self.q6] = sol

            self.valueChanged()

            self.coppeliaSetJointsRotations(self.q1, self.q2, self.q3, self.q4, self.q5, self.q6)

        else:
            QMessageBox(QMessageBox.Critical, "Cinemática", "Error de cinemática",
                        QMessageBox.Ok, self).open()

    def pushButton_pushButtonABBXUp_onClicked(self):
        _x = self.x + self.ui.horizontalSliderABBPasos.value()

        sol = self.abbengine.inverseKinematic(_x, self.y, self.z, self.a, self.b, self.c)

        if sol is not None:
            self.x = _x

            [self.q1, self.q2, self.q3, self.q4, self.q5, self.q6] = sol

            self.valueChanged()

            self.coppeliaSetJointsRotations(self.q1, self.q2, self.q3, self.q4, self.q5, self.q6)

        else:
            QMessageBox(QMessageBox.Critical, "Cinemática", "Error de cinemática",
                        QMessageBox.Ok, self).open()

    def pushButton_pushButtonABBYDown_onClicked(self):
        _y = self.y - self.ui.horizontalSliderABBPasos.value()

        sol = self.abbengine.inverseKinematic(self.x, _y, self.z, self.a, self.b, self.c)

        if sol is not None:
            self.y = _y

            [self.q1, self.q2, self.q3, self.q4, self.q5, self.q6] = sol

            self.valueChanged()

            self.coppeliaSetJointsRotations(self.q1, self.q2, self.q3, self.q4, self.q5, self.q6)

        else:
            QMessageBox(QMessageBox.Critical, "Cinemática", "Error de cinemática",
                        QMessageBox.Ok, self).open()

    def pushButton_pushButtonABBYUp_onClicked(self):
        _y = self.y + self.ui.horizontalSliderABBPasos.value()

        sol = self.abbengine.inverseKinematic(self.x, _y, self.z, self.a, self.b, self.c)

        if sol is not None:
            self.y = _y

            [self.q1, self.q2, self.q3, self.q4, self.q5, self.q6] = sol

            self.valueChanged()

            self.coppeliaSetJointsRotations(self.q1, self.q2, self.q3, self.q4, self.q5, self.q6)

        else:
            QMessageBox(QMessageBox.Critical, "Cinemática", "Error de cinemática",
                        QMessageBox.Ok, self).open()

    def pushButton_pushButtonABBZDown_onClicked(self):
        _z = self.z - self.ui.horizontalSliderABBPasos.value()

        sol = self.abbengine.inverseKinematic(self.x, self.y, _z, self.a, self.b, self.c)

        if sol is not None:
            self.z = _z

            [self.q1, self.q2, self.q3, self.q4, self.q5, self.q6] = sol

            self.valueChanged()

            self.coppeliaSetJointsRotations(self.q1, self.q2, self.q3, self.q4, self.q5, self.q6)

        else:
            QMessageBox(QMessageBox.Critical, "Cinemática", "Error de cinemática",
                        QMessageBox.Ok, self).open()

    def pushButton_pushButtonABBZUp_onClicked(self):
        _z = self.z + self.ui.horizontalSliderABBPasos.value()

        sol = self.abbengine.inverseKinematic(self.x, self.y, _z, self.a, self.b, self.c)

        if sol is not None:
            self.z = _z

            [self.q1, self.q2, self.q3, self.q4, self.q5, self.q6] = sol

            self.valueChanged()

            self.coppeliaSetJointsRotations(self.q1, self.q2, self.q3, self.q4, self.q5, self.q6)

        else:
            QMessageBox(QMessageBox.Critical, "Cinemática", "Error de cinemática",
                        QMessageBox.Ok, self).open()

    def pushButton_pushButtonABBADown_onClicked(self):
        _a = self.a - np.deg2rad(self.ui.horizontalSliderABBPasos.value())

        if _a < (-np.pi * 2.0):
            _a += np.pi * 2.0

        sol = self.abbengine.inverseKinematic(self.x, self.y, self.z, _a, self.b, self.c)

        if sol is not None:
            self.a = _a

            [self.q1, self.q2, self.q3, self.q4, self.q5, self.q6] = sol

            self.valueChanged()

            self.coppeliaSetJointsRotations(self.q1, self.q2, self.q3, self.q4, self.q5, self.q6)

        else:
            QMessageBox(QMessageBox.Critical, "Cinemática", "Error de cinemática",
                        QMessageBox.Ok, self).open()

    def pushButton_pushButtonABBAUp_onClicked(self):
        _a = self.a + np.deg2rad(self.ui.horizontalSliderABBPasos.value())

        if _a > (np.pi * 2.0):
            _a -= np.pi * 2.0

        sol = self.abbengine.inverseKinematic(self.x, self.y, self.z, _a, self.b, self.c)

        if sol is not None:
            self.a = _a

            [self.q1, self.q2, self.q3, self.q4, self.q5, self.q6] = sol

            self.valueChanged()

            self.coppeliaSetJointsRotations(self.q1, self.q2, self.q3, self.q4, self.q5, self.q6)

        else:
            QMessageBox(QMessageBox.Critical, "Cinemática", "Error de cinemática",
                        QMessageBox.Ok, self).open()

    def pushButton_pushButtonABBBDown_onClicked(self):
        _b = self.b - np.deg2rad(self.ui.horizontalSliderABBPasos.value())

        if _b < (-np.pi * 2.0):
            _b += np.pi * 2.0

        sol = self.abbengine.inverseKinematic(self.x, self.y, self.z, self.a, _b, self.c)

        if sol is not None:
            self.b = _b

            [self.q1, self.q2, self.q3, self.q4, self.q5, self.q6] = sol

            self.valueChanged()

            self.coppeliaSetJointsRotations(self.q1, self.q2, self.q3, self.q4, self.q5, self.q6)

        else:
            QMessageBox(QMessageBox.Critical, "Cinemática", "Error de cinemática",
                        QMessageBox.Ok, self).open()

    def pushButton_pushButtonABBBUp_onClicked(self):
        _b = self.b + np.deg2rad(self.ui.horizontalSliderABBPasos.value())

        if _b > (np.pi * 2.0):
            _b -= np.pi * 2.0

        sol = self.abbengine.inverseKinematic(self.x, self.y, self.z, self.a, _b, self.c)

        if sol is not None:
            self.b = _b

            [self.q1, self.q2, self.q3, self.q4, self.q5, self.q6] = sol

            self.valueChanged()

            self.coppeliaSetJointsRotations(self.q1, self.q2, self.q3, self.q4, self.q5, self.q6)

        else:
            QMessageBox(QMessageBox.Critical, "Cinemática", "Error de cinemática",
                        QMessageBox.Ok, self).open()

    def pushButton_pushButtonABBCDown_onClicked(self):
        _c = self.c - np.deg2rad(self.ui.horizontalSliderABBPasos.value())

        sol = self.abbengine.inverseKinematic(self.x, self.y, self.z, self.a, self.b, _c)

        if sol is not None:
            self.c = _c

            [self.q1, self.q2, self.q3, self.q4, self.q5, self.q6] = sol

            self.valueChanged()

            self.coppeliaSetJointsRotations(self.q1, self.q2, self.q3, self.q4, self.q5, self.q6)

        else:
            QMessageBox(QMessageBox.Critical, "Cinemática", "Error de cinemática",
                        QMessageBox.Ok, self).open()

    def pushButton_pushButtonABBCUp_onClicked(self):
        _c = self.c + np.deg2rad(self.ui.horizontalSliderABBPasos.value())

        sol = self.abbengine.inverseKinematic(self.x, self.y, self.z, self.a, self.b, _c)

        if sol is not None:
            self.c = _c

            [self.q1, self.q2, self.q3, self.q4, self.q5, self.q6] = sol

            self.valueChanged()

            self.coppeliaSetJointsRotations(self.q1, self.q2, self.q3, self.q4, self.q5, self.q6)

        else:
            QMessageBox(QMessageBox.Critical, "Cinemática", "Error de cinemática",
                        QMessageBox.Ok, self).open()

    def pushButton_pushButtonABBCapturarPunto_onClicked(self):
        self.trajectory.append([self.x, self.y, self.z, self.a, self.b, self.c])

        self.ui.labelABBCantidadPuntos.setText(str(len(self.trajectory)))

    def pushButton_pushButtonABBLimpiarTrayectoria_onClicked(self):
        self.trajectory.clear()

        self.ui.labelABBCantidadPuntos.setText(str(len(self.trajectory)))

    def pushButton_pushButtonABBEjecutarTrayectoria_onClicked(self):
        if len(self.trajectory) > 0:
            traj_home = [
                self.abbengine.linTrajectory(self.trajectory[len(self.trajectory) - 1][0],
                                             self.trajectory[len(self.trajectory) - 1][1],
                                             self.trajectory[len(self.trajectory) - 1][2],
                                             self.trajectory[len(self.trajectory) - 1][3],
                                             self.trajectory[len(self.trajectory) - 1][4],
                                             self.trajectory[len(self.trajectory) - 1][5],
                                             self.p_home[0], self.p_home[1], self.p_home[2],
                                             self.p_home[3], self.p_home[4], self.p_home[5],
                                             100)
            ]

            for i in traj_home:
                for j in i:
                    self.coppeliaSetJointsRotations(j[0], j[1], j[2], j[3], j[4], j[5])
                    time.sleep(0.01)

            QMessageBox(QMessageBox.Warning, "Trayectoria", "COI alcanzado",
                        QMessageBox.Ok, self).exec_()

            traj = [
                self.abbengine.linTrajectory(self.p_home[0], self.p_home[1], self.p_home[2],
                                             self.p_home[3], self.p_home[4], self.p_home[5],
                                             self.trajectory[0][0],
                                             self.trajectory[0][1],
                                             self.trajectory[0][2],
                                             self.trajectory[0][3],
                                             self.trajectory[0][4],
                                             self.trajectory[0][5],
                                             100)
            ]

            for i in range(0, len(self.trajectory) - 1):
                traj.append(self.abbengine.linTrajectory(self.trajectory[i][0], self.trajectory[i][1],
                                                         self.trajectory[i][2], self.trajectory[i][3],
                                                         self.trajectory[i][4], self.trajectory[i][5],
                                                         self.trajectory[i + 1][0], self.trajectory[i + 1][1],
                                                         self.trajectory[i + 1][2], self.trajectory[i + 1][3],
                                                         self.trajectory[i + 1][4], self.trajectory[i + 1][5],
                                                         100))

            for i in traj:
                for j in i:
                    self.coppeliaSetJointsRotations(j[0], j[1], j[2], j[3], j[4], j[5])
                    time.sleep(0.01)

        else:
            QMessageBox(QMessageBox.Warning, "Trayectoria", "No hay puntos para ejecutar",
                        QMessageBox.Ok, self).open()

    def pushButton_pushButtonABBGuardarTrayectoria_onClicked(self):
        file_name = QFileDialog.getSaveFileName(self, "Guardar trayectoria", os.getcwd() + "/trajs",
                                                "*.traj")

        file = open(file_name[0] + ".txt", "w+")

        for i in range(0, len(self.trajectory)):
            file.write("Punto %d\r\n" % (i + 1))
            file.write("%s\r\n" % self.trajectory[i])

        file.close()

    def valueChanged(self):
        self.ui.labelABBPosicionX.setText(str("%.2f" % self.x))
        self.ui.labelABBPosicionY.setText(str("%.2f" % self.y))
        self.ui.labelABBPosicionZ.setText(str("%.2f" % self.z))

        self.ui.labelABBOrientacionA.setText(str("%.2f" % np.rad2deg(self.a)))
        self.ui.labelABBOrientacionB.setText(str("%.2f" % np.rad2deg(self.b)))
        self.ui.labelABBOrientacionC.setText(str("%.2f" % np.rad2deg(self.c)))

        self.ui.labelABBQ1.setText(str("%.2f" % np.rad2deg(self.q1)))
        self.ui.labelABBQ2.setText(str("%.2f" % np.rad2deg(self.q2)))
        self.ui.labelABBQ3.setText(str("%.2f" % np.rad2deg(self.q3)))
        self.ui.labelABBQ4.setText(str("%.2f" % np.rad2deg(self.q4)))
        self.ui.labelABBQ5.setText(str("%.2f" % np.rad2deg(self.q5)))
        self.ui.labelABBQ6.setText(str("%.2f" % np.rad2deg(self.q6)))

    def coppeliaConnect(self, ip, port):
        # Cerramos todas las conexiones
        self.coppeliaDisconnect()

        # Guardamos el ID de conexión
        self.coppeliaIdClient = sim.simxStart(ip, port, True, True, 1000, 5)

        return self.coppeliaIdClient != -1

    def coppeliaDisconnect(self):
        sim.simxFinish(-1)

    def coppeliaIsConnected(self):
        return sim.simxGetConnectionId(self.coppeliaIdClient) != -1

    def coppeliaBegin(self):
        ret_code_joint1, self.coppeliaJoint1Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaJoint1Name, sim.simx_opmode_blocking)

        ret_code_joint2, self.coppeliaJoint2Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaJoint2Name, sim.simx_opmode_blocking)

        ret_code_joint3, self.coppeliaJoint3Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaJoint3Name, sim.simx_opmode_blocking)

        ret_code_joint4, self.coppeliaJoint4Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaJoint4Name, sim.simx_opmode_blocking)

        ret_code_joint5, self.coppeliaJoint5Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaJoint5Name, sim.simx_opmode_blocking)

        ret_code_joint6, self.coppeliaJoint6Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaJoint6Name, sim.simx_opmode_blocking)

        return (ret_code_joint1 or ret_code_joint2 or ret_code_joint3 or
                ret_code_joint4 or ret_code_joint5 or ret_code_joint6) == sim.simx_return_ok

    def coppeliaSetJointsRotations(self, q1, q2, q3, q4, q5, q6):
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaJoint1Handle,
                                 q1, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaJoint2Handle,
                                 q2, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaJoint3Handle,
                                 q3, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaJoint4Handle,
                                 q4, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaJoint5Handle,
                                 q5, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaJoint6Handle,
                                 q6, sim.simx_opmode_oneshot)