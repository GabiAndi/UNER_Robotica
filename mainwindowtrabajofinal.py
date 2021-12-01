from PySide2.QtWidgets import QMainWindow, QMessageBox
from PySide2.QtCore import Signal

from ui_mainwindowtrabajofinal import Ui_MainWindowTrabajoFinal

from abbengine import ABBEngine

import sim
import numpy as np
import time


class MainWindowTrabajoFinal(QMainWindow):
    # Eventos
    closed = Signal()

    def __init__(self, parent=None):
        super(MainWindowTrabajoFinal, self).__init__(parent)

        self.ui = Ui_MainWindowTrabajoFinal()
        self.ui.setupUi(self)

        # Posiciones de interes
        self.p_home = [374.0, 0.0, 630.0, np.pi, np.pi / 2.0, 0.0]

        # Valores iniciales
        self.coppeliaIdClient = -1

        # Brazo 1
        # Valores actuales del robot
        self.abb1_q1 = 0.0
        self.abb1_q2 = 0.0
        self.abb1_q3 = 0.0
        self.abb1_q4 = 0.0
        self.abb1_q5 = 0.0
        self.abb1_q6 = 0.0

        self.abb1_x = self.p_home[0]
        self.abb1_y = self.p_home[1]
        self.abb1_z = self.p_home[2]

        self.abb1_a = self.p_home[3]
        self.abb1_b = self.p_home[4]
        self.abb1_c = self.p_home[5]

        # Juntas
        self.coppeliaABB1Joint1Name = "IRB120_joint_1"
        self.coppeliaABB1Joint2Name = "IRB120_joint_2"
        self.coppeliaABB1Joint3Name = "IRB120_joint_3"
        self.coppeliaABB1Joint4Name = "IRB120_joint_4"
        self.coppeliaABB1Joint5Name = "IRB120_joint_5"
        self.coppeliaABB1Joint6Name = "IRB120_joint_6"

        self.coppeliaABB1Joint1Handle = None
        self.coppeliaABB1Joint2Handle = None
        self.coppeliaABB1Joint3Handle = None
        self.coppeliaABB1Joint4Handle = None
        self.coppeliaABB1Joint5Handle = None
        self.coppeliaABB1Joint6Handle = None

        # Vector de trayectorias
        self.abb1Instructions = []

        # Motor de cinematica
        self.abb2Engine = ABBEngine()

        # Brazo 2
        # Valores actuales del robot
        self.abb2_q1 = 0.0
        self.abb2_q2 = 0.0
        self.abb2_q3 = 0.0
        self.abb2_q4 = 0.0
        self.abb2_q5 = 0.0
        self.abb2_q6 = 0.0

        self.abb2_x = self.p_home[0]
        self.abb2_y = self.p_home[1]
        self.abb2_z = self.p_home[2]

        self.abb2_a = self.p_home[3]
        self.abb2_b = self.p_home[4]
        self.abb2_c = self.p_home[5]

        # Juntas
        self.coppeliaABB2Joint1Name = "IRB120_joint_1_2"
        self.coppeliaABB2Joint2Name = "IRB120_joint_2_2"
        self.coppeliaABB2Joint3Name = "IRB120_joint_3_2"
        self.coppeliaABB2Joint4Name = "IRB120_joint_4_2"
        self.coppeliaABB2Joint5Name = "IRB120_joint_5_2"
        self.coppeliaABB2Joint6Name = "IRB120_joint_6_2"

        self.coppeliaABB2Joint1Handle = None
        self.coppeliaABB2Joint2Handle = None
        self.coppeliaABB2Joint3Handle = None
        self.coppeliaABB2Joint4Handle = None
        self.coppeliaABB2Joint5Handle = None
        self.coppeliaABB2Joint6Handle = None

        # Vector de trayectorias
        self.abb2Instructions = []

        # Motor de cinematica
        self.abb2Engine = ABBEngine()

        # Eventos de botones
        self.ui.pushButtonIniciar.clicked.connect(self.pushButtonIniciar_onCliked)

    def closeEvent(self, event):
        self.coppeliaDisconnect()

        self.closed.emit()

    def pushButtonIniciar_onCliked(self):
        if self.coppeliaConnect("127.0.0.1", 19999):
            if self.coppeliaBegin():
                QMessageBox(QMessageBox.Information, "Escena", "Escena iniciada correctamente",
                            QMessageBox.Ok, self).exec_()

                self.start()

            else:
                QMessageBox(QMessageBox.Critical, "Escena", "Error en la configuracion",
                            QMessageBox.Ok, self).exec_()

        else:
            QMessageBox(QMessageBox.Critical, "Escena", "Error de conexión",
                        QMessageBox.Ok, self).exec_()

    def start(self):
        # Establecemos todas las instrucciones
        self.abb1Instructions.append()

    def abb1ExecTrajectory(self, traj):
        for i in traj:
            self.coppeliaSetJointsRotations(i[0], i[1], i[2], i[3], i[4], i[5])
            self.abb1_q1, self.abb1_q2, self.abb1_q3, self.abb1_q4, self.abb1_q5, self.abb1_q6 = \
                i[0], i[1], i[2], i[3], i[4], i[5]
            time.sleep(0.01)

    def abb1ExecPtpTrajectory(self, x, y, z, a, b, c):
        traj = self.abbengine.ptpTrajectory(self.abb1_x, self.abb1_y, self.abb1_z,
                                            self.abb1_a, self.abb1_b, self.abb1_c,
                                            x, y, z, a, b, c,
                                            100)

        if traj:
            self.execTrajectory(traj)
            self.abb1_x, self.abb1_y, self.abb1_z, self.abb1_a, self.abb1_b, self.abb1_c = x, y, z, a, b, c
            self.valueChanged()

            return True

        else:
            return False

    def abb1ExecLinTrajectory(self, x, y, z, a, b, c):
        traj = self.abbengine.linTrajectory(self.abb1_x, self.abb1_y, self.abb1_z,
                                            self.abb1_a, self.abb1_b, self.abb1_c,
                                            x, y, z, a, b, c,
                                            100)

        if traj:
            self.execTrajectory(traj)
            self.abb1_x, self.abb1_y, self.abb1_z, self.abb1_a, self.abb1_b, self.abb1_c = x, y, z, a, b, c
            self.valueChanged()

            return True

        else:
            return False

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
        # Para el brazo 1
        ret_code_abb1_joint1, self.coppeliaABB1Joint1Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaABB1Joint1Name, sim.simx_opmode_blocking)

        ret_code_abb1_joint2, self.coppeliaABB1Joint2Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaABB1Joint2Name, sim.simx_opmode_blocking)

        ret_code_abb1_joint3, self.coppeliaABB1Joint3Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaABB1Joint3Name, sim.simx_opmode_blocking)

        ret_code_abb1_joint4, self.coppeliaABB1Joint4Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaABB1Joint4Name, sim.simx_opmode_blocking)

        ret_code_abb1_joint5, self.coppeliaABB1Joint5Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaABB1Joint5Name, sim.simx_opmode_blocking)

        ret_code_abb1_joint6, self.coppeliaABB1Joint6Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaABB1Joint6Name, sim.simx_opmode_blocking)

        # Para el brazo 2
        ret_code_abb2_joint1, self.coppeliaABB2Joint1Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaABB2Joint1Name, sim.simx_opmode_blocking)

        ret_code_abb2_joint2, self.coppeliaABB2Joint2Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaABB2Joint2Name, sim.simx_opmode_blocking)

        ret_code_abb2_joint3, self.coppeliaABB2Joint3Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaABB2Joint3Name, sim.simx_opmode_blocking)

        ret_code_abb2_joint4, self.coppeliaABB2Joint4Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaABB2Joint4Name, sim.simx_opmode_blocking)

        ret_code_abb2_joint5, self.coppeliaABB2Joint5Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaABB2Joint5Name, sim.simx_opmode_blocking)

        ret_code_abb2_joint6, self.coppeliaABB2Joint6Handle = \
            sim.simxGetObjectHandle(self.coppeliaIdClient, self.coppeliaABB2Joint6Name, sim.simx_opmode_blocking)

        return (ret_code_abb1_joint1 or ret_code_abb1_joint2 or ret_code_abb1_joint3 or
                ret_code_abb1_joint4 or ret_code_abb1_joint5 or ret_code_abb1_joint6 or
                ret_code_abb2_joint1 or ret_code_abb2_joint2 or ret_code_abb2_joint3 or
                ret_code_abb2_joint4 or ret_code_abb2_joint5 or ret_code_abb2_joint6) == sim.simx_return_ok

    def coppeliaABB1SetJointsRotations(self, q1, q2, q3, q4, q5, q6):
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB1Joint1Handle,
                                 q1, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB1Joint2Handle,
                                 q2, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB1Joint3Handle,
                                 q3, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB1Joint4Handle,
                                 q4, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB1Joint5Handle,
                                 q5, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB1Joint6Handle,
                                 q6, sim.simx_opmode_oneshot)

    def coppeliaABB2SetJointsRotations(self, q1, q2, q3, q4, q5, q6):
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB2Joint1Handle,
                                 q1, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB2Joint2Handle,
                                 q2, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB2Joint3Handle,
                                 q3, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB2Joint4Handle,
                                 q4, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB2Joint5Handle,
                                 q5, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB2Joint6Handle,
                                 q6, sim.simx_opmode_oneshot)

    def coppeliaABB1SetVacuumGripper(self, active):
        sim.simxSetInt32Signal(self.coppeliaIdClient, "VacuumGripper_active", active, sim.simx_opmode_oneshot)

    def coppeliaABB2SetVacuumGripper(self, active):
        sim.simxSetInt32Signal(self.coppeliaIdClient, "VacuumGripperB_active", active, sim.simx_opmode_oneshot)
