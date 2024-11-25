#################################################################
#                       BEI EasyMile                            #
#    Physical Model : Vinicius MORENO SANCHES et Nizar ISMAT    #
#################################################################

import math
import numpy as np
import matplotlib.pyplot as plt
from enumFile.enum_class import *
import json


class VehicleState:
    """
    This class initiates the states of the vehicle with vel = theta = pos_x = pos_y = 0.
    With this parameters, it is possible to simulate a step starting from any initial condition.
    """

    def __init__(self):
        self.vel = 0  # meters per second
        self.theta = 0  # radians
        self.pos_x = 0  # meters
        self.pos_y = 0  # meters
        self.accel = 0  # m/s^2
        self.throttle = 0  # [-1,1]
        self.steer = 0  # rad


class Graph:
    """
    This class initiates the arrays to plot the graphics in the end of the simulation.
    """

    def __init__(self):
        self.pos_x_plot = []
        self.pos_y_plot = []
        self.theta_plot = []
        self.vel_plot = []
        self.t_plot = []
        self.brake_force_plot = []


class SimulationPlotter:
    """
    This class is static just to plot the simulation results.
    """

    @staticmethod
    def plot_graphs(graph):
        plt.figure()

        plt.subplot(2, 2, 1)
        plt.plot(graph.pos_x_plot, graph.pos_y_plot, marker="o")
        plt.title("Vehicle Trajectory", fontsize=10)
        plt.xlabel("Position X (m)", fontsize=10)
        plt.ylabel("Position Y (m)", fontsize=10)
        plt.grid(True)

        plt.subplot(2, 2, 2)
        plt.plot(graph.t_plot, graph.vel_plot, marker="o")
        plt.title("Velocity", fontsize=10)
        plt.xlabel("Time (s)", fontsize=10)
        plt.ylabel("Velocity (Km/h)", fontsize=10)
        plt.grid(True)

        plt.subplot(2, 2, 3)
        plt.plot(graph.t_plot, graph.theta_plot, marker="o")
        plt.title("Orientation", fontsize=10)
        plt.xlabel("Time (s)", fontsize=10)
        plt.ylabel("Theta (º)", fontsize=10)
        plt.grid(True)

        plt.subplots_adjust(hspace=0.5)

        plt.show()


class PhysicsModel:
    def __init__(self, ts):
        """
        The PhysicsModel class is used to modelling all equations of our vehicle.
        Input :
            - ts: sample time (seconds).
            - params_file: json file with the parameters of the vehicle
        """
        params_file = "Model/vehicle_parameters.json"  # JSON file
        self.ts = ts
        self.load_params(params_file)

    def load_params(self, params_file):
        with open(params_file, "r") as file:
            params = json.load(file)

        self.length = params["length"]
        self.width = params["width"]
        self.height = params["height"]
        self.m = params["mass"]
        self.torque = params["torque"]
        self.wheel_radius = params["wheel_radius"]
        self.b = params["friction_coefficient"]
        self.air_density = params["air_density"]
        self.area = self.width * self.length
        self.Cd = params["Cd"]
        self.threshold_stop = params["threshold_stop"]

    def simulate_step(self, steer, throttle, direction, vehicle_state):
        """
        This function simulates one iteration based on the previous state of the vehicle and command values.
        Input:
            - steer: steering angle (rad),
            - throttle: mechanism governing fuel flow to the engine (if throttle = 1, the acceleration pedal is
            completely pressed; if throttle = 0, nothing is being pressed; if throttle = -1, the braking pedal
            is completely pressed),
            - direction: direction of the vehicle (forward = Direction.FORWARD or backward = Direction.BACKWARD),
            - vehicle_state: previous vehicle state (object VehicleState()).
        Output:
            - vehicle_state: current vehicle state (object VehicleState()).
        """
        # Verify if the entries are valid
        if throttle > 1:
            throttle = 1
            print("ATTENTION! The throttle cannot be greater than 1.")
            print("Throttle has been limited to 1")
        if throttle < -1:
            throttle = -1
            print("ATTENTION! The throttle cannot be smaller than -1.")
            print("Throttle has been limited to -1")
        if steer > math.pi / 5:
            steer = math.pi / 5
            print("ATTENTION! The steering angle cannot be greater than pi/5.")
            print("Steering angle has been limited to pi/5")
        if steer < -math.pi / 5:
            steer = -math.pi / 5
            print("ATTENTION! The steering angle cannot be smaller than -pi/5.")
            print("Steering angle has been limited to -pi/5")

        # Space-state equations
        x_dot = vehicle_state.vel * math.cos(vehicle_state.theta)
        y_dot = vehicle_state.vel * math.sin(vehicle_state.theta)
        theta_dot = vehicle_state.vel / self.length * math.tan(steer)

        # Calculate acceleration based in Motor Force, Friction Force, Aerodynamic Force and Braking Force
        accel = (1 / self.m) * (
                self.torque * throttle / self.wheel_radius
                - self.b * vehicle_state.vel
                - (1 / 2) * self.air_density * self.area * self.Cd * vehicle_state.vel ** 2)

        if throttle < 0:
            # brake force based on friction and other resisting forces
            friction_force = self.b * vehicle_state.vel  # Frictional force
            air_resistance = (
                0.5 * self.air_density * self.Cd * self.area * vehicle_state.vel**2
            )  # Aerodynamic resistance
            total_resistance = friction_force + air_resistance  # Total resistive force

            #  net brake force
            net_brake_force = -accel * self.m - total_resistance

            #  brake deceleration
            brake_deceleration = net_brake_force / self.m if net_brake_force > 0 else 0
        else:
            brake_deceleration = 0

        #Braking model
        deceleration = brake_deceleration

        # Adding deceleration (braking) force
        if direction == Direction.FORWARD:
            accel = accel - deceleration
        else:
            accel = -accel + deceleration

        # Limits of acceleration when the vehicle is already stoped
        if vehicle_state.vel == 0 and throttle < 0:
            accel = 0

        # Integral of space-state variables
        vehicle_state.pos_x += x_dot * self.ts  # Position X (m)
        vehicle_state.pos_y += y_dot * self.ts  # Position Y (m)
        vehicle_state.theta += theta_dot * self.ts  # Vehicle Orientation (rad)
        vehicle_state.vel += accel * self.ts  # Velocity (m/s)
        vehicle_state.accel = accel  # Acceleration (m/s^2)
        vehicle_state.throttle = throttle  # Throttle between [-1,1]
        vehicle_state.steer = steer  # Steering angle (rad)

        # Saturation in 20 km/h and pi/5 radians
        MAX_FORWARD_VELOCITY = 20 / 3.6
        MAX_BACKWARD_VELOCITY = -10 / 3.6
        if direction == Direction.FORWARD:
            vehicle_state.vel = min(vehicle_state.vel, MAX_FORWARD_VELOCITY)
            vehicle_state.vel = max(vehicle_state.vel, 0)
        else:
            vehicle_state.vel = max(vehicle_state.vel, MAX_BACKWARD_VELOCITY)
            vehicle_state.vel = min(vehicle_state.vel, 0)

        if (
            vehicle_state.vel == MAX_FORWARD_VELOCITY
            or vehicle_state.vel == MAX_BACKWARD_VELOCITY
        ):
            vehicle_state.accel = 0

        return vehicle_state


# Generic simulation test (vehicle turning in a circle)
if __name__ == "__main__":
    t = 20  # Complete simulation time (s)
    Ts = 0.01  # Sample time (s)

    # Create a command array for throttle and steering angle
    throttle = np.concatenate(
        [
            np.repeat(1, 2000 // 5),
            np.repeat(1, 2 * 2000 // 5),
            np.repeat(1, 2 * 2000 // 5),
        ]
    )
    steering_angle = np.concatenate(
        [
            np.repeat(-math.pi / 5, 2 * 2000 // 5),
            np.repeat(-math.pi / 5, 2000 // 5),
            np.repeat(-math.pi / 5, 2 * 2000 // 5),
        ]
    )

    # Creates the object of the model, the vehicle state and the log arrays
    simulation = PhysicsModel(Ts)
    vehicle_state = VehicleState()
    graph = Graph()

    # Initiates the vehicle state with an initial condition of position (x,y) = (20,20)
    vehicle_state.pos_x = 20
    vehicle_state.pos_y = 20
    vehicle_state.vel = 20

    # Simulation loop
    for i in range(
        int(t / Ts)
    ):  # use the number of iterations necessary to achieve the complete simulation time
        simulation.simulate_step(
            steer=steering_angle[i],
            throttle=throttle[i],
            direction=Direction.FORWARD,
            vehicle_state=vehicle_state,
        )

        # Concatenate the data into the plot arrays
        graph.pos_x_plot.append(vehicle_state.pos_x)
        graph.pos_y_plot.append(vehicle_state.pos_y)
        graph.theta_plot.append(math.degrees(vehicle_state.theta) % 360)
        graph.vel_plot.append(
            vehicle_state.vel * 3.6
        )  # converts to km/h to show in the graphic

    # Create a time array to plot the graphics
    for i in range(len(graph.pos_x_plot)):
        graph.t_plot.append(i * Ts)

    SimulationPlotter.plot_graphs(graph)

    print("Final Position: ", vehicle_state.pos_x, vehicle_state.pos_y)
    print("Final Orientation: ", vehicle_state.theta)
    print("Final Velocity: ", vehicle_state.vel)
