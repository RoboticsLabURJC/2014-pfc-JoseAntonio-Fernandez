from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap

class Form(QWidget):
    def __init__(self, parent=None):
        super(Form, self).__init__(parent)


        self.submitButton = QPushButton("Nop button")

        self.label = QLabel()
        pixmap = QPixmap("plazaMayor500x500.png")
        self.label.setPixmap(pixmap)


        buttonLayout1 = QVBoxLayout()
        buttonLayout1.addWidget(self.submitButton)
        buttonLayout1.addWidget(self.label)





        self.submitButton.clicked.connect(self.submitContact)

        mainLayout = QGridLayout()
        mainLayout.addLayout(buttonLayout1, 0, 1)

        self.setLayout(mainLayout)
        self.setWindowTitle("AUVCommander")

    def submitContact(self):
        x = 0


if __name__ == '__main__':
    import sys

    app = QApplication(sys.argv)

    screen = Form()
    screen.show()

    sys.exit(app.exec_())