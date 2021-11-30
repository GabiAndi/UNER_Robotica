from PySide2.QtCore import Qt
from PySide2.QtWidgets import QMainWindow

from ui_mainwindow import Ui_MainWindow
from mainwindowabbpanel import MainWindowABBPanel


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Conexion de eventos
        self.ui.pushButtonCinematica.clicked.connect(self.__mainWindowCinematicaOpen)

        # Ventanas
        self.mainwindowCinematica = None

    def __mainWindowCinematicaOpen(self):
        if self.mainwindowCinematica is None:
            self.mainwindowCinematica = MainWindowABBPanel(self)
            self.mainwindowCinematica.closed.connect(self.__mainWindowCinematicaClose)
            self.mainwindowCinematica.setWindowModality(Qt.ApplicationModal)
            self.mainwindowCinematica.show()

    def __mainWindowCinematicaClose(self):
        self.mainwindowCinematica = None

