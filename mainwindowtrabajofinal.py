from PySide2.QtWidgets import QMainWindow, QMessageBox
from PySide2.QtCore import Signal, QObject, QThread

from ui_mainwindowtrabajofinal import Ui_MainWindowTrabajoFinal

from abbengine import ABBEngine

import sim
import numpy as np
import time
import csv


class MainWindowTrabajoFinal(QMainWindow):
    # Eventos
    closed = Signal()

    # Comunicacion con el hilo
    sceneStart = Signal()
    sceneStop = Signal()

    def __init__(self, parent=None):
        super(MainWindowTrabajoFinal, self).__init__(parent)

        self.ui = Ui_MainWindowTrabajoFinal()
        self.ui.setupUi(self)

        # Controlador de escena
        self.sceneThread = QThread(self)
        self.scene = TrabajoFinal()

        self.scene.moveToThread(self.sceneThread)

        self.sceneThread.start()

        # Eventos de botones
        self.ui.pushButtonIniciar.clicked.connect(self.pushButtonIniciar_onCliked)
        self.ui.pushButtonDetener.clicked.connect(self.pushButtonDetener_onCliked)

        # Conexiones
        self.sceneStart.connect(self.scene.start)
        self.sceneStop.connect(self.scene.stop)

        self.scene.sceneStarted.connect(self.sceneStarted)
        self.scene.sceneError.connect(self.sceneError)
        self.scene.sceneStoped.connect(self.sceneStop)

    def closeEvent(self, event):
        self.sceneStop.emit()

        self.sceneThread.exit()
        self.sceneThread.wait(1000)

        self.closed.emit()

    def pushButtonIniciar_onCliked(self):
        self.sceneStart.emit()

    def pushButtonDetener_onCliked(self):
        self.sceneStop.emit()

    def sceneStarted(self):
        QMessageBox(QMessageBox.Information, "Escena", "La escena se inicio correctamente",
                    QMessageBox.Ok, self).exec_()

    def sceneError(self, error):
        if error == 0:
            QMessageBox(QMessageBox.Critical, "Escena", "Error de conexión",
                        QMessageBox.Ok, self).exec_()

        elif error == 1:
            QMessageBox(QMessageBox.Critical, "Escena", "Error de configuracion",
                        QMessageBox.Ok, self).exec_()

    def sceneStoped(self):
        QMessageBox(QMessageBox.Warning, "Escena", "La escena se detuvo correctamente",
                    QMessageBox.Ok, self).exec_()


class TrabajoFinal(QObject):
    # Señales
    sceneStarted = Signal()
    sceneError = Signal(int)
    sceneStoped = Signal()

    def __init__(self, parent=None):
        super(TrabajoFinal, self).__init__(parent)

        # Bandera que detiene la ejecucion
        self.programInterrupt = False

        # Posiciones de interes
        self.p_home = [374.0 + 67.0, 0.0, 630.0, np.pi, np.pi / 2.0, 0.0]

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
        self.abb1Trajectory = []

        # Motor de cinematica
        self.abb1Engine = ABBEngine()

        # Herramienta
        self.abb1Engine.tool = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 67.0],
                [0.0, 0.0, 0.0, 1.0]
            ]
        )

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
        self.abb2Trajectory = []

        # Motor de cinematica
        self.abb2Engine = ABBEngine()

        # Herramienta
        self.abb2Engine.tool = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 67.0],
                [0.0, 0.0, 0.0, 1.0]
            ]
        )

        # Autito
        # Parametros del autito
        self.carWheelsAxisDistanceL = 1
        self.carWheelsRadius = 1

        # Objetos de la escena de Coppelia
        # Ruedas
        self.carLeftJoint = None
        self.carRigthJoint = None

        # Sensores
        self.carSensor = [None, None, None, None, None, None, None, None]
        self.carSensorValue = []

    def start(self):
        if self.coppeliaConnect("127.0.0.1", 19999):
            if self.coppeliaBegin():
                self.sceneStarted.emit()

                self.exec()

            else:
                self.sceneError.emit(1)

        else:
            self.sceneError.emit(0)

    def stop(self):
        self.coppeliaDisconnect()

        self.sceneStoped.emit()

    def exec(self):
        # Establecemos las trayectorias
        # Brazo 1
        """if not self.abb1LoadTrajectory():
            return

        self.abb1ExecLinTrajectory(self.p_home[0], self.p_home[1], self.p_home[2],
                                   self.p_home[3], self.p_home[4], self.p_home[5])

        for i in range(0, len(self.abb1Trajectory)):
            self.abb1ExecLinTrajectory(self.abb1Trajectory[i][0], self.abb1Trajectory[i][1],
                                       self.abb1Trajectory[i][2], self.abb1Trajectory[i][3],
                                       self.abb1Trajectory[i][4], self.abb1Trajectory[i][5])

            if (i == 3) or (i == 10) or (i == 18):
                self.coppeliaABB1SetVacuumGripper(1)

            if (i == 6) or (i == 14) or (i == 22):
                self.coppeliaABB1SetVacuumGripper(0)"""

        # Autito
        car_complete = False

        while not car_complete:
            # Leemos el valor de los sensores
            self.carSensorValue = self.carReadSensors()

            if self.carSensorValue:
                # Aca tengo que analizar el valor de los sensores
                # self.carSensorValue

                time.sleep(1)

    def abb1ExecTrajectory(self, traj):
        for i in traj:
            self.coppeliaABB1SetJointsRotations(i[0], i[1], i[2], i[3], i[4], i[5])
            self.abb1_q1, self.abb1_q2, self.abb1_q3, self.abb1_q4, self.abb1_q5, self.abb1_q6 = \
                i[0], i[1], i[2], i[3], i[4], i[5]
            time.sleep(0.01)

    def abb1ExecPtpTrajectory(self, x, y, z, a, b, c):
        traj = self.abb1Engine.ptpTrajectory(self.abb1_x, self.abb1_y, self.abb1_z,
                                             self.abb1_a, self.abb1_b, self.abb1_c,
                                             x, y, z, a, b, c,
                                             100)

        if traj:
            self.abb1ExecTrajectory(traj)
            self.abb1_x, self.abb1_y, self.abb1_z, self.abb1_a, self.abb1_b, self.abb1_c = x, y, z, a, b, c

            return True

        else:
            return False

    def abb1ExecLinTrajectory(self, x, y, z, a, b, c):
        traj = self.abb1Engine.linTrajectory(self.abb1_x, self.abb1_y, self.abb1_z,
                                             self.abb1_a, self.abb1_b, self.abb1_c,
                                             x, y, z, a, b, c,
                                             100)

        if traj:
            self.abb1ExecTrajectory(traj)
            self.abb1_x, self.abb1_y, self.abb1_z, self.abb1_a, self.abb1_b, self.abb1_c = x, y, z, a, b, c

            return True

        else:
            return False

    def abb1LoadTrajectory(self):
        file = open("trajs/ABB1.traj", "r")

        if file:
            a = list(csv.reader(file, delimiter=','))

            self.abb1Trajectory.clear()

            for row in a:
                _row = []

                for i in range(0, 6):
                    _row.append(float(row[i]))

                self.abb1Trajectory.append(_row)

            file.close()

            return True

        else:
            return False

    def carReadSensors(self):
        values = []

        # Valores de los sensores
        for i in range(0, 8):
            ret_code, res, value = sim.simxReadVisionSensor(self.coppeliaIdClient,
                                                            self.carSensor[i],
                                                            sim.simx_opmode_streaming)

            if value:
                values.append(value[0][0])

            else:
                return []

        return values

    def carSensorMeansValue(self, sensors):
        means = []

        for i in sensors:
            means.append(np.mean(i))

        return means

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

        # Handles del autito
        ret_code_car_left_joint, self.carLeftJoint = sim.simxGetObjectHandle(self.coppeliaIdClient,
                                                                             "Pioneer_p3dx_leftMotor",
                                                                             sim.simx_opmode_blocking)

        ret_code_car_right_joint, self.carRigthJoint = sim.simxGetObjectHandle(self.coppeliaIdClient,
                                                                               "Pioneer_p3dx_rightMotor",
                                                                               sim.simx_opmode_blocking)

        # Handles de los sensores
        ret_code_car_sensor0, self.carSensor[0] = sim.simxGetObjectHandle(self.coppeliaIdClient,
                                                                          "Sensor0",
                                                                          sim.simx_opmode_blocking)

        ret_code_car_sensor1, self.carSensor[1] = sim.simxGetObjectHandle(self.coppeliaIdClient,
                                                                          "Sensor1",
                                                                          sim.simx_opmode_blocking)

        ret_code_car_sensor2, self.carSensor[2] = sim.simxGetObjectHandle(self.coppeliaIdClient,
                                                                          "Sensor2",
                                                                          sim.simx_opmode_blocking)

        ret_code_car_sensor3, self.carSensor[3] = sim.simxGetObjectHandle(self.coppeliaIdClient,
                                                                          "Sensor3",
                                                                          sim.simx_opmode_blocking)

        ret_code_car_sensor4, self.carSensor[4] = sim.simxGetObjectHandle(self.coppeliaIdClient,
                                                                          "Sensor4",
                                                                          sim.simx_opmode_blocking)

        ret_code_car_sensor5, self.carSensor[5] = sim.simxGetObjectHandle(self.coppeliaIdClient,
                                                                          "Sensor5",
                                                                          sim.simx_opmode_blocking)

        ret_code_car_sensor6, self.carSensor[6] = sim.simxGetObjectHandle(self.coppeliaIdClient,
                                                                          "Sensor6",
                                                                          sim.simx_opmode_blocking)

        ret_code_car_sensor7, self.carSensor[7] = sim.simxGetObjectHandle(self.coppeliaIdClient,
                                                                          "Sensor7",
                                                                          sim.simx_opmode_blocking)

        return (ret_code_abb1_joint1 or ret_code_abb1_joint2 or ret_code_abb1_joint3 or
                ret_code_abb1_joint4 or ret_code_abb1_joint5 or ret_code_abb1_joint6 or
                ret_code_abb2_joint1 or ret_code_abb2_joint2 or ret_code_abb2_joint3 or
                ret_code_abb2_joint4 or ret_code_abb2_joint5 or ret_code_abb2_joint6 or
                ret_code_car_sensor0 or ret_code_car_sensor1 or ret_code_car_sensor2 or
                ret_code_car_sensor3 or ret_code_car_sensor4 or ret_code_car_sensor5 or
                ret_code_car_sensor6 or ret_code_car_sensor7) == sim.simx_return_ok

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
