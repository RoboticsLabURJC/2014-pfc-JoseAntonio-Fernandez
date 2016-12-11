
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap

class Form(QWidget):
    def __init__(self, uri, parent=None):
        super(Form, self).__init__(parent)


        self.submitButton = QPushButton("Nop button")

        self.label = QLabel()
        pixmap = QPixmap(uri)
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



#TODO Interfaz grafico
#TODO ver como muestro el recorrido de la aeronave

#TODO Ver actitud, mapa y posteriormente imagen
#TODO Marcar en un mapa waypoints para generar un path a la aeronave