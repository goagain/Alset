import carla
import PyQt5
import MainApplication
import sys
from PyQt5.QtWidgets import QApplication

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MainApplication.MainApplication()
    ex.show()
    app.exit(app.exec_())
