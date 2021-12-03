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

        self.scene.sceneCarObsSensorChanged.connect(self.sceneCarObsSensorChanged)
        self.scene.sceneCarSensorChanged.connect(self.sceneCarSensorChanged)

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

    def sceneCarSensorChanged(self, values):
        self.ui.labelCarSensor0.setText(str("%.4f" % values[0]))
        self.ui.labelCarSensor1.setText(str("%.4f" % values[1]))
        self.ui.labelCarSensor2.setText(str("%.4f" % values[2]))
        self.ui.labelCarSensor3.setText(str("%.4f" % values[3]))
        self.ui.labelCarSensor4.setText(str("%.4f" % values[4]))
        self.ui.labelCarSensor5.setText(str("%.4f" % values[5]))

    def sceneCarObsSensorChanged(self, values):
        self.ui.labelCarObsSensor1.setText(str("%.4f" % values[0]))
        self.ui.labelCarObsSensor8.setText(str("%.4f" % values[7]))
        self.ui.labelCarObsSensor9.setText(str("%.4f" % values[8]))
        self.ui.labelCarObsSensor10.setText(str("%.4f" % values[9]))
        self.ui.labelCarObsSensor11.setText(str("%.4f" % values[10]))
        self.ui.labelCarObsSensor12.setText(str("%.4f" % values[11]))
        self.ui.labelCarObsSensor13.setText(str("%.4f" % values[12]))
        self.ui.labelCarObsSensor14.setText(str("%.4f" % values[13]))
        self.ui.labelCarObsSensor15.setText(str("%.4f" % values[14]))
        self.ui.labelCarObsSensor16.setText(str("%.4f" % values[15]))


class TrabajoFinal(QObject):
    # Señales
    sceneStarted = Signal()
    sceneError = Signal(int)
    sceneStoped = Signal()

    sceneCarObsSensorChanged = Signal(list)
    sceneCarSensorChanged = Signal(list)

    def __init__(self, parent=None):
        super(TrabajoFinal, self).__init__(parent)

        # Bandera que detiene la ejecucion
        self.programInterrupt = False

        # Posiciones de interes
        self.abb1_p_home = [374.0 + 67.0, 0.0, 630.0, np.pi, np.pi / 2.0, 0.0]
        self.abb2_p_home = [200.0, 0.0, 630.0, np.pi, np.pi / 2.0, 0.0]

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

        self.abb1_x = self.abb1_p_home[0]
        self.abb1_y = self.abb1_p_home[1]
        self.abb1_z = self.abb1_p_home[2]

        self.abb1_a = self.abb1_p_home[3]
        self.abb1_b = self.abb1_p_home[4]
        self.abb1_c = self.abb1_p_home[5]

        # Juntas
        self.coppeliaABB1JointHandle = [None, None, None, None, None, None]

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

        self.abb2_x = self.abb1_p_home[0]
        self.abb2_y = self.abb1_p_home[1]
        self.abb2_z = self.abb1_p_home[2]

        self.abb2_a = self.abb1_p_home[3]
        self.abb2_b = self.abb1_p_home[4]
        self.abb2_c = self.abb1_p_home[5]

        # Juntas
        self.coppeliaABB2JointHandle = [None, None, None, None, None, None]

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
        self.carObsSensor = []

        for i in range(0, 16):
            self.carObsSensor.append(None)

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
        if not self.abb1LoadTrajectory():
            return

        # Brazo 2
        if not self.abb2LoadTrajectory():
            return

        # Movimientos al home
        self.abb1ExecLinTrajectory(self.abb1_p_home[0], self.abb1_p_home[1], self.abb1_p_home[2],
                                   self.abb1_p_home[3], self.abb1_p_home[4], self.abb1_p_home[5])

        self.abb2ExecLinTrajectory(self.abb2_p_home[0], self.abb2_p_home[1], self.abb2_p_home[2],
                                   self.abb2_p_home[3], self.abb2_p_home[4], self.abb2_p_home[5])

        # Movimientos del brazo 1
        for i in range(0, len(self.abb1Trajectory)):
            self.abb1ExecLinTrajectory(self.abb1Trajectory[i][0], self.abb1Trajectory[i][1],
                                       self.abb1Trajectory[i][2], self.abb1Trajectory[i][3],
                                       self.abb1Trajectory[i][4], self.abb1Trajectory[i][5])

            if (i == 3) or (i == 10) or (i == 18):
                self.coppeliaABB1SetVacuumGripper(1)

            if (i == 6) or (i == 14) or (i == 22):
                self.coppeliaABB1SetVacuumGripper(0)

        # Autito
        car_complete = False

        # Seguidor de linea
        # Ponderacion
        lin_pond = [2.0, 1.0, 0.5, 0.3, 0.3, 0.5, 1.0, 2.0]

        lin_kp = 2.5
        lin_kd = 2.5

        lin_error_prev = 0.0

        # Esquive de obstaculos
        box_kp = 2.5

        # Maquina de estados para la solución del problema
        car_state = 0

        while not car_complete:
            # Leemos los valores del ultrasonico
            car_obs_sensor_value = self.carReadObsSensors()

            # Enviamos los valores
            send_obs_val = []

            for i in car_obs_sensor_value:
                send_obs_val.append(i[0])

            self.sceneCarObsSensorChanged.emit(send_obs_val)

            # Leemos el valor de los sensores de vision
            car_sensor_value = self.carReadSensors()

            if car_sensor_value:
                self.sceneCarSensorChanged.emit(car_sensor_value)

            # Maquina de estados
            # El auto esta siguiendo la linea
            if car_state == 0:
                if car_sensor_value:
                    # PID
                    lin_error = 0.0

                    lin_error += car_sensor_value[0] * lin_pond[0] - car_sensor_value[7] * lin_pond[7]
                    lin_error += car_sensor_value[1] * lin_pond[1] - car_sensor_value[6] * lin_pond[6]
                    lin_error += car_sensor_value[2] * lin_pond[2] - car_sensor_value[5] * lin_pond[5]
                    lin_error += car_sensor_value[3] * lin_pond[3] - car_sensor_value[4] * lin_pond[4]

                    lin_error /= 8

                    lin_correction = lin_error * lin_kp + lin_error_prev * lin_kd

                    self.carSetVelocity(-0.5, -lin_correction)

                    # Valores del PID
                    lin_error_prev = lin_error

                # El auto detecto un obtaculo en frente
                for i in range(10, 16):
                    if car_obs_sensor_value[i][1]:
                        if car_obs_sensor_value[i][0] < 0.15:
                            self.carSetVelocity(0.0, 0.0)

                            car_state = 1

            # El auto tiene que analizar hacia que lado evitara el obstaculo
            elif car_state == 1:
                direction_pond = [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0]

                # Direccion
                direction = 0.0

                direction += car_obs_sensor_value[10][0] * direction_pond[0] + \
                             car_obs_sensor_value[15][0] * direction_pond[5]
                direction += car_obs_sensor_value[11][0] * direction_pond[1] + \
                             car_obs_sensor_value[14][0] * direction_pond[4]
                direction += car_obs_sensor_value[12][0] * direction_pond[2] + \
                             car_obs_sensor_value[13][0] * direction_pond[3]

                # El auto deberia esquivar por la izquierda
                if direction < 0.0:
                    car_state = 10

                else:
                    car_state = 20

            # Esquivando el auto por la derecha
            elif car_state == 10:
                self.carSetVelocity(0.0, 0.5)
                time.sleep(5.0)

                car_state = 11

            elif car_state == 11:
                # P
                box_error = 0.125 - car_obs_sensor_value[7][0]
                box_correction = box_error * box_kp
                self.carSetVelocity(-0.5, box_correction)

                # Medicion de la linea
                for i in range(0, 8):
                    if car_sensor_value[i] < 0.2:
                        car_state = 0

            # Esquivando el auto por la izquierda
            elif car_state == 20:
                self.carSetVelocity(0.0, -0.5)
                time.sleep(5.0)

                car_state = 21

            elif car_state == 21:
                # P
                box_error = 0.125 - car_obs_sensor_value[7][0]
                box_correction = box_error * box_kp
                self.carSetVelocity(-0.5, -box_correction)

                # Medicion de la linea
                for i in range(0, 8):
                    if car_sensor_value[i] < 0.2:
                        car_state = 0

            car_sim = sim.simxGetInt32Signal(self.coppeliaIdClient, "car_finished", sim.simx_opmode_oneshot)

            if car_sim[0] == sim.simx_return_ok:
                if car_sim[1] == 1:
                    self.carSetVelocity(0.0, 0.0)
                    car_complete = True

            time.sleep(0.1)

        # Brazo 2
        # Posicionamos las cajas como corresponde
        sim.simxSetInt32Signal(self.coppeliaIdClient, "box_teleport", 1, sim.simx_opmode_oneshot)

        # Movimientos del brazo 2
        for i in range(0, len(self.abb2Trajectory)):
            self.abb2ExecLinTrajectory(self.abb2Trajectory[i][0], self.abb2Trajectory[i][1],
                                       self.abb2Trajectory[i][2], self.abb2Trajectory[i][3],
                                       self.abb2Trajectory[i][4], self.abb2Trajectory[i][5])

            if (i == 0) or (i == 13) or (i == 28):
                self.coppeliaABB2SetVacuumGripper(1)

            if (i == 8) or (i == 22) or (i == 34):
                self.coppeliaABB2SetVacuumGripper(0)

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

    def abb2ExecTrajectory(self, traj):
        for i in traj:
            self.coppeliaABB2SetJointsRotations(i[0], i[1], i[2], i[3], i[4], i[5])
            self.abb2_q1, self.abb2_q2, self.abb2_q3, self.abb2_q4, self.abb2_q5, self.abb2_q6 = \
                i[0], i[1], i[2], i[3], i[4], i[5]
            time.sleep(0.01)

    def abb2ExecPtpTrajectory(self, x, y, z, a, b, c):
        traj = self.abb2Engine.ptpTrajectory(self.abb2_x, self.abb2_y, self.abb2_z,
                                             self.abb2_a, self.abb2_b, self.abb2_c,
                                             x, y, z, a, b, c,
                                             100)

        if traj:
            self.abb2ExecTrajectory(traj)
            self.abb2_x, self.abb2_y, self.abb2_z, self.abb2_a, self.abb2_b, self.abb2_c = x, y, z, a, b, c

            return True

        else:
            return False

    def abb2ExecLinTrajectory(self, x, y, z, a, b, c):
        traj = self.abb2Engine.linTrajectory(self.abb2_x, self.abb2_y, self.abb2_z,
                                             self.abb2_a, self.abb2_b, self.abb2_c,
                                             x, y, z, a, b, c,
                                             100)

        if traj:
            self.abb2ExecTrajectory(traj)
            self.abb2_x, self.abb2_y, self.abb2_z, self.abb2_a, self.abb2_b, self.abb2_c = x, y, z, a, b, c

            return True

        else:
            return False

    def abb2LoadTrajectory(self):
        file = open("trajs/ABB2.traj", "r")

        if file:
            a = list(csv.reader(file, delimiter=','))

            self.abb2Trajectory.clear()

            for row in a:
                _row = []

                for i in range(0, 6):
                    _row.append(float(row[i]))

                self.abb2Trajectory.append(_row)

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
                _val = np.mean(value[0])

                values.append(_val)

            else:
                values.append(0.0)

        return values

    def carReadObsSensors(self):
        values = []

        # Valores de los sensores
        for i in range(0, 16):
            ret = sim.simxReadProximitySensor(
                self.coppeliaIdClient, self.carObsSensor[i], sim.simx_opmode_oneshot
            )

            val = np.sqrt(np.power(ret[2][0], 2) + np.power(ret[2][1], 2) + np.power(ret[2][2], 2))

            if val > 1.0:
                val = 1.0

            values.append([val, ret[1]])

        return values

    def carSetVelocity(self, vel, omega):
        # Obtenemos la velocidad de cada rueda para las velocidades
        omega_left = (vel - self.carWheelsAxisDistanceL * omega) / self.carWheelsRadius
        omega_right = (vel + self.carWheelsAxisDistanceL * omega) / self.carWheelsRadius

        self.coppeliaCarSetWheelsVelocity(omega_left, omega_right)

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
        for i in range(0, 6):
            ret_code, self.coppeliaABB1JointHandle[i] = sim.simxGetObjectHandle(
                self.coppeliaIdClient, "IRB120_joint_" + str(i + 1), sim.simx_opmode_blocking
            )

            if ret_code != sim.simx_return_ok:
                return False

        # Para el brazo 2
        for i in range(0, 6):
            ret_code, self.coppeliaABB2JointHandle[i] = sim.simxGetObjectHandle(
                self.coppeliaIdClient, "IRB120_joint_" + str(i + 1) + "_2", sim.simx_opmode_blocking
            )

            if ret_code != sim.simx_return_ok:
                return False

        # Handles del autito
        ret_code, self.carLeftJoint = sim.simxGetObjectHandle(self.coppeliaIdClient,
                                                              "Pioneer_p3dx_leftMotor",
                                                              sim.simx_opmode_blocking)

        if ret_code != sim.simx_return_ok:
            return False

        ret_code, self.carRigthJoint = sim.simxGetObjectHandle(self.coppeliaIdClient,
                                                               "Pioneer_p3dx_rightMotor",
                                                               sim.simx_opmode_blocking)

        if ret_code != sim.simx_return_ok:
            return False

        # Handles de los sensores de linea
        for i in range(0, 8):
            ret_code, self.carSensor[i] = sim.simxGetObjectHandle(self.coppeliaIdClient,
                                                                  "Sensor" + str(i),
                                                                  sim.simx_opmode_blocking)

            if ret_code != sim.simx_return_ok:
                return False

        # Handles de los sensores de parede
        for i in range(0, 16):
            ret_code, self.carObsSensor[i] = sim.simxGetObjectHandle(
                self.coppeliaIdClient,
                "Pioneer_p3dx_ultrasonicSensor" + str(i + 1),
                sim.simx_opmode_blocking
            )

            if ret_code != sim.simx_return_ok:
                return False

        return True

    def coppeliaABB1SetJointsRotations(self, q1, q2, q3, q4, q5, q6):
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB1JointHandle[0],
                                 q1, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB1JointHandle[1],
                                 q2, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB1JointHandle[2],
                                 q3, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB1JointHandle[3],
                                 q4, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB1JointHandle[4],
                                 q5, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB1JointHandle[5],
                                 q6, sim.simx_opmode_oneshot)

    def coppeliaABB2SetJointsRotations(self, q1, q2, q3, q4, q5, q6):
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB2JointHandle[0],
                                 q1, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB2JointHandle[1],
                                 q2, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB2JointHandle[2],
                                 q3, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB2JointHandle[3],
                                 q4, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB2JointHandle[4],
                                 q5, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.coppeliaIdClient, self.coppeliaABB2JointHandle[5],
                                 q6, sim.simx_opmode_oneshot)

    def coppeliaABB1SetVacuumGripper(self, active):
        sim.simxSetInt32Signal(self.coppeliaIdClient, "VacuumGripper_active", active, sim.simx_opmode_oneshot)

    def coppeliaABB2SetVacuumGripper(self, active):
        sim.simxSetInt32Signal(self.coppeliaIdClient, "VacuumGripperB_active", active, sim.simx_opmode_oneshot)

    def coppeliaCarSetWheelsVelocity(self, wheel_left, wheel_right):
        sim.simxSetJointTargetVelocity(self.coppeliaIdClient, self.carLeftJoint, wheel_left,
                                       sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.coppeliaIdClient, self.carRigthJoint, wheel_right,
                                       sim.simx_opmode_oneshot)
