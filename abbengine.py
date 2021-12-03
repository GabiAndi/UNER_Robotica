import numpy as np
import roboticstoolbox as rtb
import spatialmath as sma


class ABBEngine:
    def __init__(self):
        # Parametros del DH
        self.MDH_a1, self.MDH_alpha1, self.MDH_d1, self.MDH_theta1 = 0.0, 0.0, 290.0, 0.0
        self.MDH_a2, self.MDH_alpha2, self.MDH_d2, self.MDH_theta2 = 0.0, -90.0, 0.0, -90.0
        self.MDH_a3, self.MDH_alpha3, self.MDH_d3, self.MDH_theta3 = 270.0, 0.0, 0.0, 0.0
        self.MDH_a4, self.MDH_alpha4, self.MDH_d4, self.MDH_theta4 = 70.0, -90.0, 302.0, 0.0
        self.MDH_a5, self.MDH_alpha5, self.MDH_d5, self.MDH_theta5 = 0.0, 90.0, 0.0, 0.0
        self.MDH_a6, self.MDH_alpha6, self.MDH_d6, self.MDH_theta6 = 0.0, -90.0, 72.0, 0.0

        # Matriz homogenea de la herramienta
        self.tool = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

    def directKinematic(self, q1, q2, q3, q4, q5, q6):
        # Calculo de la junta 1
        rotx = np.array([[1.0, 0.0, 0.0, 0.0],
                         [0.0, np.cos(np.deg2rad(self.MDH_alpha1)), -np.sin(np.deg2rad(self.MDH_alpha1)), 0.0],
                         [0.0, np.sin(np.deg2rad(self.MDH_alpha1)), np.cos(np.deg2rad(self.MDH_alpha1)), 0.0],
                         [0.0, 0.0, 0.0, 1.0]])
        tx = np.array([[1.0, 0.0, 0.0, self.MDH_a1], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        rotz = np.array(
            [[np.cos(np.deg2rad(self.MDH_theta1 + q1)), -np.sin(np.deg2rad(self.MDH_theta1 + q1)), 0, 0],
             [np.sin(np.deg2rad(self.MDH_theta1 + q1)), np.cos(np.deg2rad(self.MDH_theta1 + q1)), 0, 0],
             [0.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 1.0]])
        tz = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, self.MDH_d1], [0.0, 0.0, 0.0, 1.0]])

        m1 = rotx @ tx @ rotz @ tz

        # Calculo de la junta 2
        rotx = np.array([[1.0, 0.0, 0.0, 0.0],
                         [0.0, np.cos(np.deg2rad(self.MDH_alpha2)), -np.sin(np.deg2rad(self.MDH_alpha2)), 0.0],
                         [0.0, np.sin(np.deg2rad(self.MDH_alpha2)), np.cos(np.deg2rad(self.MDH_alpha2)), 0.0],
                         [0.0, 0.0, 0.0, 1.0]])
        tx = np.array([[1.0, 0.0, 0.0, self.MDH_a2], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        rotz = np.array(
            [[np.cos(np.deg2rad(self.MDH_theta2 + q2)), -np.sin(np.deg2rad(self.MDH_theta2 + q2)), 0.0, 0.0],
             [np.sin(np.deg2rad(self.MDH_theta2 + q2)), np.cos(np.deg2rad(self.MDH_theta2 + q2)), 0.0, 0.0],
             [0.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 1.0]])
        tz = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, self.MDH_d2], [0.0, 0.0, 0.0, 1.0]])

        m2 = rotx @ tx @ rotz @ tz

        # Calculo de la junta 3
        rotx = np.array([[1.0, 0.0, 0.0, 0.0],
                         [0.0, np.cos(np.deg2rad(self.MDH_alpha3)), -np.sin(np.deg2rad(self.MDH_alpha3)), 0.0],
                         [0.0, np.sin(np.deg2rad(self.MDH_alpha3)), np.cos(np.deg2rad(self.MDH_alpha3)), 0.0],
                         [0.0, 0.0, 0.0, 1.0]])
        tx = np.array([[1.0, 0.0, 0.0, self.MDH_a3], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        rotz = np.array(
            [[np.cos(np.deg2rad(self.MDH_theta3 + q3)), -np.sin(np.deg2rad(self.MDH_theta3 + q3)), 0, 0],
             [np.sin(np.deg2rad(self.MDH_theta3 + q3)), np.cos(np.deg2rad(self.MDH_theta3 + q3)), 0, 0],
             [0.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 1.0]])
        tz = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, self.MDH_d3], [0.0, 0.0, 0.0, 1.0]])

        m3 = rotx @ tx @ rotz @ tz

        # Calculo de la junta 4
        rotx = np.array([[1.0, 0.0, 0.0, 0.0],
                         [0.0, np.cos(np.deg2rad(self.MDH_alpha4)), -np.sin(np.deg2rad(self.MDH_alpha4)), 0.0],
                         [0.0, np.sin(np.deg2rad(self.MDH_alpha4)), np.cos(np.deg2rad(self.MDH_alpha4)), 0.0],
                         [0.0, 0.0, 0.0, 1.0]])
        tx = np.array([[1.0, 0.0, 0.0, self.MDH_a4], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        rotz = np.array(
            [[np.cos(np.deg2rad(self.MDH_theta4 + q4)), -np.sin(np.deg2rad(self.MDH_theta4 + q4)), 0.0, 0.0],
             [np.sin(np.deg2rad(self.MDH_theta4 + q4)), np.cos(np.deg2rad(self.MDH_theta4 + q4)), 0.0, 0.0],
             [0.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 1.0]])
        tz = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, self.MDH_d4], [0.0, 0.0, 0.0, 1.0]])

        m4 = rotx @ tx @ rotz @ tz

        # Calculo de la junta 5
        rotx = np.array([[1.0, 0.0, 0.0, 0.0],
                         [0.0, np.cos(np.deg2rad(self.MDH_alpha5)), -np.sin(np.deg2rad(self.MDH_alpha5)), 0.0],
                         [0.0, np.sin(np.deg2rad(self.MDH_alpha5)), np.cos(np.deg2rad(self.MDH_alpha5)), 0.0],
                         [0.0, 0.0, 0.0, 1.0]])
        tx = np.array([[1.0, 0.0, 0.0, self.MDH_a5], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        rotz = np.array(
            [[np.cos(np.deg2rad(self.MDH_theta5 + q5)), -np.sin(np.deg2rad(self.MDH_theta5 + q5)), 0.0, 0.0],
             [np.sin(np.deg2rad(self.MDH_theta5 + q5)), np.cos(np.deg2rad(self.MDH_theta5 + q5)), 0.0, 0.0],
             [0.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 1.0]])
        tz = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, self.MDH_d5], [0.0, 0.0, 0.0, 1.0]])

        m5 = rotx @ tx @ rotz @ tz

        # Calculo de la junta 6
        rotx = np.array([[1.0, 0.0, 0.0, 0.0],
                         [0.0, np.cos(np.deg2rad(self.MDH_alpha6)), -np.sin(np.deg2rad(self.MDH_alpha6)), 0.0],
                         [0.0, np.sin(np.deg2rad(self.MDH_alpha6)), np.cos(np.deg2rad(self.MDH_alpha6)), 0.0],
                         [0.0, 0.0, 0.0, 1.0]])
        tx = np.array([[1.0, 0.0, 0.0, self.MDH_a6], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        rotz = np.array(
            [[np.cos(np.deg2rad(self.MDH_theta6 + q6)), -np.sin(np.deg2rad(self.MDH_theta6 + q6)), 0.0, 0.0],
             [np.sin(np.deg2rad(self.MDH_theta6 + q6)), np.cos(np.deg2rad(self.MDH_theta6 + q6)), 0.0, 0.0],
             [0.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 1.0]])
        tz = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, self.MDH_d6], [0.0, 0.0, 0.0, 1.0]])

        m6 = rotx @ tx @ rotz @ tz

        # Devolucion del efector final
        m0 = m1 @ m2 @ m3 @ m4 @ m5 @ m6 @ self.tool

        # Posicion
        x = m0[0, 3]
        y = m0[1, 3]
        z = m0[2, 3]

        # Orientacion
        orientation = np.array([
            [m0[0, 0], m0[0, 1], m0[0, 2]],
            [m0[1, 0], m0[1, 1], m0[1, 2]],
            [m0[2, 0], m0[2, 1], m0[2, 2]]
        ])

        [a, b, c] = self.inverseEuler(orientation)

        return [x, y, z, a, b, c]

    def inverseKinematic(self, x, y, z, a, b, c):
        # Obtenemos la matriz de orientación
        orientation_target = self.directEuler(a, b, c)

        # Formamos la matriz objetivo
        at = np.array([
            [
                orientation_target[0, 0],
                orientation_target[0, 1],
                orientation_target[0, 2],
                x
            ],
            [
                orientation_target[1, 0],
                orientation_target[1, 1],
                orientation_target[1, 2],
                y
            ],
            [
                orientation_target[2, 0],
                orientation_target[2, 1],
                orientation_target[2, 2],
                z
            ],
            [
                0.0,
                0.0,
                0.0,
                1.0
            ]
        ])

        return self.inverseKinematic2(at)

    def inverseKinematic2(self, at):
        # Traemos el efector final a la muñeca
        # De la herramienta a la brida
        atool = np.linalg.inv(self.tool)

        # De la brida a la muñeca
        ab = np.array([[1.0, 0.0, 0.0, 0.0],
                       [0.0, 1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0, -self.MDH_d6],
                       [0.0, 0.0, 0.0, 1.0]])

        am = at @ atool @ ab

        # Descomponemos los parametros
        nx = am[0, 0]
        ny = am[1, 0]
        nz = am[2, 0]

        ox = am[0, 1]
        oy = am[1, 1]
        oz = am[2, 1]

        ax = am[0, 2]
        ay = am[1, 2]
        az = am[2, 2]

        px = am[0, 3]
        py = am[1, 3]
        pz = am[2, 3]

        # Cálculo de la junta 1
        r = np.sqrt(np.power(px, 2) + np.power(py, 2))

        q1 = np.arctan2(py, px)

        # No se puede rotar la junta
        if (q1 < np.deg2rad(-165.0)) or (q1 > np.deg2rad(165.0)):
            return None

        # Cálculo de la junta 2
        zq2 = pz - 290.0

        beta = np.arctan2(zq2, r)

        r35 = np.sqrt(np.power(302.0, 2) + np.power(70.0, 2))
        r25 = np.sqrt(np.power(r, 2) + np.power(zq2, 2))

        cos_alpha = (np.power(270.0, 2) + np.power(r25, 2) - np.power(r35, 2)) / (2.0 * 270.0 * r25)

        # Si el ángulo da complejo
        sqrt_value = 1 - np.power(cos_alpha, 2)

        if sqrt_value < 0.0:
            return None

        sin_alpha = np.sqrt(sqrt_value)

        alpha = np.arctan2(sin_alpha, cos_alpha)

        q2 = np.pi / 2.0 - alpha - beta

        # No se puede rotar la junta
        if (q2 < np.deg2rad(-110.0)) or (q2 > np.deg2rad(110.0)):
            return None

        # Cálculo de la junta 3
        gamma = np.arctan2(302.0, 70.0)

        sin_phi = (r25 * sin_alpha) / r35
        cos_phi = (np.power(270.0, 2) + np.power(r35, 2) - np.power(r25, 2)) / (2.0 * 270.0 * r35)

        phi = np.arctan2(sin_phi, cos_phi)

        q3 = np.pi - phi - gamma

        # No se puede rotar la junta
        if (q3 < np.deg2rad(-110.0)) or (q3 > np.deg2rad(70.0)):
            return None

        # Cálculo de la junta 5
        q5 = -np.arccos(
            ax * np.cos(q1) * np.cos(q2 + q3) +
            ay * np.sin(q1) * np.cos(q2 + q3) -
            az * np.sin(q2 + q3)
        )

        # Si q5 es cero practicamente estamos en una singularidad
        if np.abs(q5) < np.deg2rad(0.001):
            q4 = 0.0
            q5 = 0.0

            # Depende el valor fijado para q4
            q6 = np.arccos(
                nx * np.cos(q1) * np.sin(q2 + q3) +
                ny * np.sin(q1) * np.sin(q2 + q3) +
                nz * np.cos(q2 + q3)
            )

        else:
            # Cálculo de la junta 4
            q4 = np.arctan2(
                ax * np.sin(q1) -
                ay * np.cos(q1),
                ax * np.cos(q1) * np.sin(q2 + q3) +
                ay * np.sin(q1) * np.sin(q2 + q3) +
                az * np.cos(q2 + q3)
            )

            # Cálculo de la junta 6
            q6 = np.arctan2(
                -(ox * np.cos(q1) * np.cos(q2 + q3) +
                  oy * np.sin(q1) * np.cos(q2 + q3) -
                  oz * np.sin(q2 + q3)),
                nx * np.cos(q1) * np.cos(q2 + q3) +
                ny * np.sin(q1) * np.cos(q2 + q3) -
                nz * np.sin(q2 + q3)
            )

            # Comprobamos si es posible orientar las juntas
            if ((q4 < np.deg2rad(-160.0)) or (q4 > np.deg2rad(160.0)) or
                    (q5 < np.deg2rad(-120.0)) or (q5 > np.deg2rad(120.0)) or
                    (q6 < np.deg2rad(-400.0)) or (q6 > np.deg2rad(400.0))):
                # Probamos la solucion inversa
                q4 -= np.deg2rad(180.0)
                q5 *= -1
                q6 -= np.deg2rad(180.0)

                # Si no es posible la rotacion se realiza en sentido contrario
                if q4 < np.deg2rad(-160.0):
                    q4 += np.deg2rad(360.0)

                    if q6 < np.deg2rad(-180.0):
                        q6 += np.deg2rad(360.0)

        # No se puede rotar las juntas
        if (q4 < np.deg2rad(-160.0)) or (q4 > np.deg2rad(160.0)):
            return None

        if (q5 < np.deg2rad(-120.0)) or (q5 > np.deg2rad(120.0)):
            return None

        if (q6 < np.deg2rad(-400.0)) or (q6 > np.deg2rad(400.0)):
            return None

        return [q1, q2, q3, q4, q5, q6]

    def directEuler(self, a, b, c):
        return np.array([
            [
                np.cos(b) * np.cos(c),
                -np.cos(b) * np.sin(c),
                np.sin(b)
            ],
            [
                np.sin(a) * np.sin(b) * np.cos(c) + np.cos(a) * np.sin(c),
                -np.sin(a) * np.sin(b) * np.sin(c) + np.cos(a) * np.cos(c),
                -np.sin(a) * np.cos(b)
            ],
            [
                -np.cos(a) * np.sin(b) * np.cos(c) + np.sin(a) * np.sin(c),
                np.cos(a) * np.sin(b) * np.sin(c) + np.sin(a) * np.cos(c),
                np.cos(a) * np.cos(b)
            ]
        ])

    def inverseEuler(self, orientation):
        return [0.0, 0.0, 0.0]

    def ptpTrajectory(self, x1, y1, z1, a1, b1, c1, x2, y2, z2, a2, b2, c2, n):
        # Comprobamos solucion de t1
        p1 = self.inverseKinematic(x1, y1, z1, a1, b1, c1)

        if not p1:
            return None

        # Comprobamos solucion de t2
        p2 = self.inverseKinematic(x2, y2, z2, a2, b2, c2)

        if not p2:
            return None

        # Interpolamos
        t = np.arange(0, 1, 1 / (n - 1))
        q = rtb.jtraj(p1, p2, t).q

        return q

    def linTrajectory(self, x1, y1, z1, a1, b1, c1, x2, y2, z2, a2, b2, c2, n):
        # Punto 1
        orientation_target = self.directEuler(a1, b1, c1)

        # Formamos la matriz objetivo
        p1 = np.array([
            [
                orientation_target[0, 0],
                orientation_target[0, 1],
                orientation_target[0, 2],
                x1
            ],
            [
                orientation_target[1, 0],
                orientation_target[1, 1],
                orientation_target[1, 2],
                y1
            ],
            [
                orientation_target[2, 0],
                orientation_target[2, 1],
                orientation_target[2, 2],
                z1
            ],
            [
                0.0,
                0.0,
                0.0,
                1.0
            ]
        ])

        t1 = sma.SE3()

        t1.n[0] = p1[0, 0]
        t1.n[1] = p1[1, 0]
        t1.n[2] = p1[2, 0]

        t1.o[0] = p1[0, 1]
        t1.o[1] = p1[1, 1]
        t1.o[2] = p1[2, 1]

        t1.a[0] = p1[0, 2]
        t1.a[1] = p1[1, 2]
        t1.a[2] = p1[2, 2]

        t1.t[0] = p1[0, 3]
        t1.t[1] = p1[1, 3]
        t1.t[2] = p1[2, 3]

        # Punto 2
        orientation_target = self.directEuler(a2, b2, c2)

        # Formamos la matriz objetivo
        p2 = np.array([
            [
                orientation_target[0, 0],
                orientation_target[0, 1],
                orientation_target[0, 2],
                x2
            ],
            [
                orientation_target[1, 0],
                orientation_target[1, 1],
                orientation_target[1, 2],
                y2
            ],
            [
                orientation_target[2, 0],
                orientation_target[2, 1],
                orientation_target[2, 2],
                z2
            ],
            [
                0.0,
                0.0,
                0.0,
                1.0
            ]
        ])

        t2 = sma.SE3()

        t2.n[0] = p2[0, 0]
        t2.n[1] = p2[1, 0]
        t2.n[2] = p2[2, 0]

        t2.o[0] = p2[0, 1]
        t2.o[1] = p2[1, 1]
        t2.o[2] = p2[2, 1]

        t2.a[0] = p2[0, 2]
        t2.a[1] = p2[1, 2]
        t2.a[2] = p2[2, 2]

        t2.t[0] = p2[0, 3]
        t2.t[1] = p2[1, 3]
        t2.t[2] = p2[2, 3]

        t = []

        # Decodificamos los valores
        for i in rtb.ctraj(t1, t2, n):
            _t = np.array([
                [i.n[0], i.o[0], i.a[0], i.t[0]],
                [i.n[1], i.o[1], i.a[1], i.t[1]],
                [i.n[2], i.o[2], i.a[2], i.t[2]],
                [0.0, 0.0, 0.0, 1.0]
            ])

            t.append(_t)

        # Generamos la cinematica inversa
        q = []

        for i in t:
            _q = self.inverseKinematic2(i)

            if _q:
                q.append(_q)

            else:
                return None

        return q
