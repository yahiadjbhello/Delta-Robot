import sys
import serial
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QPushButton,
    QLineEdit, QMessageBox, QHBoxLayout
)
from PyQt5.QtCore import Qt

class DeltaControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.serial_port = None

    def initUI(self):
        self.setWindowTitle("Delta Robot - Real-Time Control")

        self.port_label = QLabel("Serial Port (e.g. COM3 or /dev/ttyUSB0):")
        self.port_input = QLineEdit()

        # === Input Fields ===
        self.kp_input = self.create_input("Kp", "40.0")
        self.ki_input = self.create_input("Ki", "0.0")
        self.kd_input = self.create_input("Kd", "0.125")
        self.radius_input = self.create_input("Radius", "0.2")
        self.freq_input = self.create_input("Frequency", "0.5")
        self.update_input = self.create_input("Update Interval", "5")

        # === Send Button ===
        self.send_button = QPushButton("Send Parameters")
        self.send_button.clicked.connect(self.send_parameters)

        # === Layout ===
        layout = QVBoxLayout()
        layout.addWidget(self.port_label)
        layout.addWidget(self.port_input)
        for label, field in self.inputs:
            layout.addWidget(label)
            layout.addWidget(field)
        layout.addWidget(self.send_button)

        self.setLayout(layout)

    def create_input(self, name, default):
        label = QLabel(f"{name}:")
        input_field = QLineEdit()
        input_field.setText(default)
        if not hasattr(self, 'inputs'):
            self.inputs = []
        self.inputs.append((label, input_field))
        return input_field

    def send_parameters(self):
        port = self.port_input.text().strip()
        if not port:
            QMessageBox.warning(self, "Error", "Please enter a valid serial port.")
            return

        try:
            if self.serial_port is None or not self.serial_port.is_open:
                self.serial_port = serial.Serial(port, 115200, timeout=1)

            # Read user input values
            params = {
                "Kp": float(self.kp_input.text()),
                "Ki": float(self.ki_input.text()),
                "Kd": float(self.kd_input.text()),
                "R": float(self.radius_input.text()),
                "F": float(self.freq_input.text()),
                "U": int(self.update_input.text())
            }

            # Format and send
            message = ";".join([f"{k}={v}" for k, v in params.items()]) + "\n"
            self.serial_port.write(message.encode())
            print("Sent:", message)

        except ValueError:
            QMessageBox.critical(self, "Input Error", "Please enter valid numeric values.")
        except serial.SerialException as e:
            QMessageBox.critical(self, "Serial Error", str(e))

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = DeltaControlGUI()
    window.show()
    sys.exit(app.exec_())
