from PySide6.QtCore import Qt
from PySide6.QtWidgets import QMainWindow

from ui_mainwindow import Ui_MainWindow

from mainwindowabbpanel import MainWindowABBPanel
from mainwindowtrabajofinal import MainWindowTrabajoFinal


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Conexion de eventos
        self.ui.pushButtonCinematica.clicked.connect(self.mainWindowCinematicaOpen)
        self.ui.pushButtonFinal.clicked.connect(self.mainWindowFinalOpen)

        # Ventanas
        self.mainwindowCinematica = None
        self.mainwindowFinal = None

    def mainWindowCinematicaOpen(self):
        if self.mainwindowCinematica is None:
            self.mainwindowCinematica = MainWindowABBPanel(self)
            self.mainwindowCinematica.closed.connect(self.mainWindowCinematicaClose)
            self.mainwindowCinematica.setWindowModality(Qt.ApplicationModal)
            self.mainwindowCinematica.show()

    def mainWindowCinematicaClose(self):
        self.mainwindowCinematica = None

    def mainWindowFinalOpen(self):
        if self.mainwindowFinal is None:
            self.mainwindowFinal = MainWindowTrabajoFinal(self)
            self.mainwindowFinal.closed.connect(self.mainWindowFinalClose)
            self.mainwindowFinal.setWindowModality(Qt.ApplicationModal)
            self.mainwindowFinal.show()

    def mainWindowFinalClose(self):
        self.mainwindowFinal = None

