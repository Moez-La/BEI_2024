import os
import matplotlib.pyplot as plt
import pyqtgraph as pg
import numpy as np
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *
from scipy.optimize import minimize
import sys
from Trajectoire import generate_trajectory

timeUpdate = 100 * 10**-3  # s


class StarterCode(QWidget):
    def __init__(self):
        super().__init__()
        self.time = 0
        self.setWindowTitle("Vehicle Control")

        # Create the main layout
        layout = QVBoxLayout(self)
        self.setLayout(layout)

        # Create the user interface widget
        self.ui = Interface()
        layout.addWidget(self.ui)

        # Create a timer for periodic updates
        self.timer = QTimer(self)

        # Connect the timer to the periodic update function
        self.timer.start(int(timeUpdate * 10**3))
        self.timer.timeout.connect(self.starterStep)
        self.timer.timeout.connect(self.ui.update_plot)

    def starterStep(self):
        """
        This function is called on each timer tick to perform periodic updates.
        """
        self.ui.time += timeUpdate

        # Autopilot: Use lateral control to calculate steering angle
        if self.ui.autopilot_is_pushed:
            steering_angle = self.ui.lateral_control()
            self.ui.theta_temp.append(self.ui.theta_temp[-1] + steering_angle * timeUpdate)  # Update orientation
            self.ui.steering_temp.append(steering_angle)

        # Manual mode: Update direction based on key presses
        if self.ui.manual_mode:
            self.ui.update_manual_control()

        # Update vehicle position based on speed and direction
        speed = 1  # Fixed speed for testing
        direction = self.ui.theta_temp[-1]
        self.ui.velocity_temp.append(speed)
        self.ui.pos_x_temp.append(self.ui.pos_x_temp[-1] + speed * np.cos(direction) * timeUpdate)
        self.ui.pos_y_temp.append(self.ui.pos_y_temp[-1] + speed * np.sin(direction) * timeUpdate)


class Interface(QWidget):
    def __init__(self):
        super().__init__()

        # Generate the trajectory using wpimath
        self.path, self.left_border, self.left_border1, self.left_border2, self.right_border = generate_trajectory()

        # Temporary storage for plotting
        self.time = 0
        self.pos_x_temp = [0]  # Initial position of the vehicle
        self.pos_y_temp = [0]
        self.theta_temp = [0]  # Initial orientation
        self.steering_temp = []
        self.velocity_temp = []

        # Control flags
        self.manual_mode = False
        self.autopilot_is_pushed = True
        self.manual_steering_angle = 0  # Steering angle in manual mode

        # Create GUI components
        self.plot_simulator = pg.PlotWidget()
        self.plot_speed = pg.PlotWidget()
        self.plot_steering = pg.PlotWidget()
        self.error_message_box = QLabel("NO ERROR")
        self.error_message_box.setStyleSheet("background-color: red")
        self.autopilot = QPushButton("Autopilot")
        self.manual_mode_button = QPushButton("Manual Mode")

        self.autopilot.setChecked(True)
        self.autopilot.setStyleSheet("background-color: green")
        self.manual_mode_button.setStyleSheet("background-color: lightgray")

        # Layout
        layout = QGridLayout(self)
        self.setLayout(layout)

        # Add widgets to layout
        layout.addWidget(self.plot_speed, 0, 0, 1, 2)  # Graphique de vitesse en haut à gauche
        layout.addWidget(self.plot_steering, 1, 0, 1, 2)  # Graphique de braquage en dessous
        layout.addWidget(self.plot_simulator, 0, 2, 2, 4)  # Simulateur occupant deux lignes et quatre colonnes
        layout.addWidget(self.error_message_box, 3, 0, 1, 2)  # Boîte d'erreur sur toute la largeur
        layout.addWidget(self.autopilot, 3, 2, 1, 2)  # Bouton autopilot au centre
        layout.addWidget(self.manual_mode_button, 3, 4, 1, 2)  # Bouton manuel à côté du bouton autopilot

        # Configure the plots
        self.plot_speed.setTitle("Vehicle Speed")
        self.plot_speed.setLabel("bottom", "Time (s)")
        self.plot_speed.setLabel("left", "Speed (m/s)")

        self.plot_steering.setTitle("Steering Angle")
        self.plot_steering.setLabel("bottom", "Time (s)")
        self.plot_steering.setLabel("left", "Angle (rad)")

        # Connect buttons
        self.autopilot.clicked.connect(self.toggle_piloting)
        self.manual_mode_button.clicked.connect(self.toggle_manual_mode)

    

    def plot_path(self):
        """Plots the trajectory on the simulator."""
        path_x, path_y = zip(*self.path)
        left_x, left_y = zip(*self.left_border)
        left_x1, left_y1 = zip(*self.left_border1)
        left_x2, left_y2 = zip(*self.left_border2)
        right_x, right_y = zip(*self.right_border)
        self.plot_simulator.plot(
            path_x, path_y,
            pen=pg.mkPen('b', width=2, style=pg.QtCore.Qt.DashLine),  # Blue line for trajectory
            name="Path"
        )
        self.plot_simulator.plot(
            left_x, left_y,
            pen=pg.mkPen('r', width=2),  # red line for left border
            name="Left_border"
        )
        self.plot_simulator.plot(
            left_x1, left_y1,
            pen=pg.mkPen('w', width=2),  # red line for left border1
            name="Left_border1"
        )
        self.plot_simulator.plot(
            left_x2, left_y2,
            pen=pg.mkPen('b', width=2,style=pg.QtCore.Qt.DashLine ),  # red line for left border2
            name="Left_border2"
        )
        self.plot_simulator.plot(
            right_x, right_y,
            pen=pg.mkPen('r', width=2),  # red line for right border
            name="Right_border"
        )
    def lateral_control(self):
        """
        Lateral control to calculate steering angle based on the trajectory.
        """
        if not self.path or not self.pos_x_temp or not self.pos_y_temp:
            return 0

        # Current position of the vehicle
        current_pos = np.array([self.pos_x_temp[-1], self.pos_y_temp[-1]])

        # Find the closest point on the trajectory
        distances = [np.linalg.norm(current_pos - np.array(p)) for p in self.path]
        closest_idx = np.argmin(distances)

        # Select the next target point
        next_idx = (closest_idx + 1) % len(self.path)
        target_point = np.array(self.path[next_idx])

        # Calculate the required steering angle
        path_vector = target_point - current_pos
        desired_angle = np.arctan2(path_vector[1], path_vector[0])

        # Angular error
        current_angle = self.theta_temp[-1]
        angle_error = desired_angle - current_angle
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))  # Normalize to [-pi, pi]

        # Proportional control for steering angle
        k_p = 2.0
        steering_angle = k_p * angle_error
        return steering_angle

    def update_manual_control(self):
        """
        Updates the vehicle orientation based on manual steering angle.
        """
        self.theta_temp[-1] += self.manual_steering_angle * timeUpdate

    def update_plot(self):
        """
        Updates the plot with the vehicle's position and trajectory.
        """
        if not self.theta_temp:
            return

        rectangle_x = np.array([-1, 1, 1, -1, -1])
        rectangle_y = np.array([-0.5, -0.5, 0.5, 0.5, -0.5])

        rotation_matrix = np.array([
            [np.cos(self.theta_temp[-1]), -np.sin(self.theta_temp[-1])],
            [np.sin(self.theta_temp[-1]), np.cos(self.theta_temp[-1])]
        ])
        rotated_rectangle = np.dot(rotation_matrix, np.vstack((rectangle_x, rectangle_y)))
        rotated_rectangle[0, :] += self.pos_x_temp[-1]
        rotated_rectangle[1, :] += self.pos_y_temp[-1]

        self.plot_simulator.clear()
        self.plot_path()  # Draw trajectory
        self.plot_simulator.plot(rotated_rectangle[0, :], rotated_rectangle[1, :])
        self.plot_simulator.plot(
            [self.pos_x_temp[-1]],
            [self.pos_y_temp[-1]],
            pen=None,
            symbol="o",
            symbolPen=None,
            symbolSize=10,
            symbolBrush="r",
            name="Vehicle"
        )

        # Update speed and steering plots
        self.plot_speed.clear()
        self.plot_speed.plot(
            [t for t in range(len(self.velocity_temp))],
            self.velocity_temp,
            pen=pg.mkPen('r', width=2)
        )
        self.plot_steering.clear()
        self.plot_steering.plot(
            [t for t in range(len(self.steering_temp))],
            self.steering_temp,
            pen=pg.mkPen('g', width=2)
        )

    def toggle_piloting(self):
        """Toggles autopilot."""
        self.autopilot_is_pushed = not self.autopilot_is_pushed
        if self.autopilot_is_pushed:
            self.autopilot.setStyleSheet("background-color: green;")
            self.manual_mode = False
            self.manual_mode_button.setStyleSheet("background-color: lightgray")
        else:
            self.autopilot_is_pushed = False
            self.autopilot.setStyleSheet("background-color: lightgray")
            self.manual_mode = True
            self.manual_mode_button.setStyleSheet("background-color: green;")

    def toggle_manual_mode(self):
        """Toggles manual mode."""
        self.manual_mode = not self.manual_mode
        if self.manual_mode:
            self.manual_mode_button.setStyleSheet("background-color: green;")
            self.autopilot_is_pushed = False
            self.autopilot.setStyleSheet("background-color: lightgray")
        else:
            self.manual_mode = False
            self.manual_mode_button.setStyleSheet("background-color: lightgray")
            self.autopilot_is_pushed = True
            self.autopilot.setStyleSheet("background-color: green;")

    def keyPressEvent(self, event):
        """
        Handle key presses for manual control.
        """
        if self.manual_mode:
            if event.key() == Qt.Key_Left:
                self.manual_steering_angle = 1  # Turn right (inverted)
            elif event.key() == Qt.Key_Right:
                self.manual_steering_angle = -1  # Turn left (inverted)

    def keyReleaseEvent(self, event):
        """
        Handle key releases for manual control.
        """
        if self.manual_mode and event.key() in [Qt.Key_Left, Qt.Key_Right]:
            self.manual_steering_angle = 0  # Stop steering


if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = StarterCode()
    widget.show()
    sys.exit(app.exec())