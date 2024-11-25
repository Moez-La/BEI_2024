#################################################################
#                       BEI EasyMile                            #
#    Interpreter: MOTELAY Hugo and SPAGNOLO Emilie              #
#################################################################

from enumFile.enum_class import (
    Starter,
    Button,
    Direction,
    Handbrake,
    Error,
    Estop,
    Piloting,
)
from Model.model_vehicle import PhysicsModel, VehicleState
import math


class Interpreter:
    def __init__(
        self,
        sample_time,
        coeff_inc_throttle_sec=0.5,
        coeff_dec_throttle_sec=0.5,
        coeff_inc_steering_sec=0.5,
        coeff_dec_steering_sec=0.5,
        limit_default_counter_throttle=3,
        limit_default_counter_steering=3,
        threshold_stop=0.027,
    ):
        self.start = Starter.OFF
        self.throttle = 0
        self.steering = 0
        self.frequency = 1 / (sample_time)
        self.default_counter_throttle = 0
        self.default_counter_steering = 0
        self.throttle_save = 0
        self.steering_save = 0
        self.coeff_inc_throttle = coeff_inc_throttle_sec / self.frequency
        self.coeff_dec_throttle = coeff_dec_throttle_sec / self.frequency
        self.coeff_inc_steering = coeff_inc_steering_sec / self.frequency
        self.coeff_dec_steering = coeff_dec_steering_sec / self.frequency
        self.limit_default_counter_throttle = limit_default_counter_throttle
        self.limit_default_counter_steering = limit_default_counter_steering
        self.handbrake = Handbrake.ACTIVATED
        self.e_stop_state = Estop.NOT_ACTIVATED
        self.mvt_direction = Direction.FORWARD
        self.physic_model = PhysicsModel(sample_time)
        self.vehicle_state = VehicleState()
        self.piloting = Piloting.AUTO
        self.threshold_stop = threshold_stop
        self.last_throttle_op = "inc"
        self.random_trajectory_counter = 0

    def set_start(self, value):
        """
        This function sets the starter mode of the vehicle.
        It stops the vehicle by also setting the handbrake/activating the emergency stop if the starter is set on "OFF"
        Input :
            enum value : 'ON' when the start button is pressed, 'OFF' when the button stop is pressed
        """
        if value != Starter.ON and value != Starter.OFF:
            return Error.InvalidValue

        if value == Starter.ON:
            if self.piloting == Piloting.MANUAL:
                self.start = Starter.ON
            else:
                return Error.HelpAutopilot

        if value == Starter.OFF and abs(self.vehicle_state.vel) <= self.threshold_stop:
            # self.handbrake = Handbrake.ACTIVATED
            self.start = Starter.OFF

        if value == Starter.OFF and abs(self.vehicle_state.vel) > self.threshold_stop:
            self.set_e_stop()

        return Error.OK

    def set_e_stop(self):
        self.e_stop_state = Estop.ACTIVATED

    def set_handbrake(self, value):
        """
        This function sets the handbrake status if the velocity alllows it
        Input :
            enum value : 'ACTIVATED' when the activate-handbrake button is pressed, 'NOT_ACTIVATED' when the desactivate-handbrake button is pressed
        """
        if value != Handbrake.ACTIVATED and value != Handbrake.NOT_ACTIVATED:
            return Error.InvalidValue

        if (
            value == Handbrake.ACTIVATED
            and abs(self.vehicle_state.vel) > self.threshold_stop
        ):
            self.handbrake = Handbrake.NOT_ACTIVATED
            return Error.BrakeErrorOff

        if abs(self.vehicle_state.vel) <= self.threshold_stop:
            self.handbrake = value

        return Error.OK

    def set_mvt_direction(self, value):
        """
        This function sets the direction of the movement if the velocity allows it
        Input :
            enum value : 'FORWARD' when the forward button is pressed, 'BACKWARD' when the backward button is pressed
        """

        if value != Direction.FORWARD and value != Direction.BACKWARD:
            return Error.InvalidValue

        if (
            value != self.mvt_direction
            and abs(self.vehicle_state.vel) > self.threshold_stop
        ):
            return Error.DirectionError

        if abs(self.vehicle_state.vel) <= self.threshold_stop:
            self.mvt_direction = value

        return Error.OK

    def set_piloting(self, value):
        """
        This function sets autopilot mode
        Input :
            enum value : 'AUTO' when the autopilot is set on, 'MANUAL' when the operator is piloting.
        """
        if value != Piloting.AUTO and value != Piloting.MANUAL:
            return Error.InvalidValue
        elif value == Piloting.AUTO:
            if abs(self.vehicle_state.vel) < self.threshold_stop:
                self.piloting = value
                self.random_trajectory_counter = 0
                return Error.OK
            else:
                return Error.SpeedAutopilot
        else:
            if self.e_stop_state == Estop.ACTIVATED:
                self.piloting = value
                return Error.OK
            else:
                return Error.HelpAutopilot

    def inc_gas(self, btn_inc_gas):
        """
        This function computes the positive throttle based on time thanks to the data frequency, when gas pedal is pressed
        Input :
            enum btn_inc_gas : 'PRESSED' when the button is pressed, else 'NOT_PRESSED'
        """
        temp_throttle = self.throttle
        if btn_inc_gas == Button.PRESSED:
            if self.default_counter_throttle == 0:
                temp_throttle += self.coeff_inc_throttle

            elif (
                self.default_counter_throttle <= self.limit_default_counter_throttle
                and self.default_counter_throttle != 0
            ):
                temp_throttle = self.throttle_save + self.coeff_inc_throttle
                self.default_counter_throttle = 0
                self.throttle_save = 0

        if temp_throttle > 1:
            temp_throttle = 1.0

        self.throttle = temp_throttle

    def dec_gas(self, btn_dec_gas):
        """
        This function computes the negative throttle based on time thanks to the data frequency, when brake is pressed
        Input :
            enum btn_dec_gas : 'PRESSED' when the button is pressed, else 'NOT_PRESSED'
        """
        temp_throttle = self.throttle
        # Test if the brake button is pressed
        if btn_dec_gas == Button.PRESSED:
            if self.default_counter_throttle == 0:
                temp_throttle -= self.coeff_dec_throttle

            elif (
                self.default_counter_throttle <= self.limit_default_counter_throttle
                and self.default_counter_throttle != 0
            ):
                temp_throttle = self.throttle_save - self.coeff_dec_throttle
                self.default_counter_throttle = 0
                self.throttle_save = 0

        if temp_throttle < -1:
            temp_throttle = -1.0

        self.throttle = temp_throttle

    def compute_throttle(self, btn_inc_gas, btn_dec_gas):
        """
        This function computes the throttle based on time thanks to the data frequency,
        It increments the throttle when the gas pedal is pressed and it decrements the throttle if the brake is pressed
        Input :
            enum btn_inc_gas : 'PRESSED' when the gas pedal is pressed, else 'NOT_PRESSED'
            enum btn_dec_gas : 'PRESSED' when the brake is pressed, else 'NOT_PRESSED'
        """

        if (btn_inc_gas != Button.PRESSED and btn_inc_gas != Button.NOT_PRESSED) or (
            btn_dec_gas != Button.PRESSED and btn_dec_gas != Button.NOT_PRESSED
        ):
            return Error.InvalidValue

        if (
            btn_inc_gas == Button.PRESSED or btn_dec_gas == Button.PRESSED
        ) and self.handbrake == Handbrake.ACTIVATED:
            return Error.BrakeErrorOn

        # Test if decrease button and increase throttle button are pressed in the same time
        if btn_dec_gas == Button.PRESSED and btn_inc_gas == Button.PRESSED:
            return Error.InvalidInput

        if btn_dec_gas == Button.PRESSED and btn_inc_gas == Button.NOT_PRESSED:
            if self.last_throttle_op == "dec":
                self.dec_gas(btn_dec_gas)

            if self.last_throttle_op == "inc":
                self.throttle = 0
                self.throttle_save = 0
                self.dec_gas(btn_dec_gas)
                self.last_throttle_op = "dec"

        if (
            btn_inc_gas == Button.PRESSED
            and btn_dec_gas == Button.NOT_PRESSED
            and self.handbrake == Handbrake.NOT_ACTIVATED
        ):
            if self.last_throttle_op == "inc":
                self.inc_gas(btn_inc_gas)

            if self.last_throttle_op == "dec":
                self.throttle = 0
                self.throttle_save = 0
                self.inc_gas(btn_inc_gas)
                self.last_throttle_op = "inc"

        if (
            self.default_counter_throttle <= self.limit_default_counter_throttle
            and btn_inc_gas == Button.NOT_PRESSED
            and btn_dec_gas == Button.NOT_PRESSED
        ):
            self.default_counter_throttle += 1
            if (
                self.default_counter_throttle == 1
            ):  # save only the first throttle value, before the missing informations
                self.throttle_save = self.throttle
            self.throttle = 0

        if (
            self.default_counter_throttle > self.limit_default_counter_throttle
            and btn_inc_gas == Button.NOT_PRESSED
            and btn_dec_gas == Button.NOT_PRESSED
        ):
            self.throttle = 0
            self.default_counter_throttle = 0
            self.throttle_save = 0

        return Error.OK

    def inc_steering(self, btn_inc_steering):
        """
        This function computes the positive steering based on time thanks to the data frequency
        Input :
            enum btn_inc_steering : 'PRESSED' when the button is pressed, else 'NOT_PRESSED'
        """

        temp_steering = self.steering

        if btn_inc_steering == Button.PRESSED:
            if self.default_counter_steering == 0:
                temp_steering += self.coeff_inc_steering

            elif (
                self.default_counter_steering <= self.limit_default_counter_steering
                and self.default_counter_steering != 0
            ):
                temp_steering = self.steering_save + self.coeff_inc_steering
                self.default_counter_steering = 0
                self.steering_save = 0

        if temp_steering > math.pi / 5:
            temp_steering = math.pi / 5

        self.steering = temp_steering

    def dec_steering(self, btn_dec_steering):
        """
        This function computes the negative steering based on time thanks to the data frequency
        Input :
            enum btn_dec_steering : 'PRESSED' when the button is pressed, else 'NOT_PRESSED'
        """
        temp_steering = self.steering
        # Test if the left button is pressed
        if btn_dec_steering == Button.PRESSED:
            if self.default_counter_steering == 0:
                temp_steering -= self.coeff_dec_steering

            elif (
                self.default_counter_steering <= self.limit_default_counter_steering
                and self.default_counter_steering != 0
            ):
                temp_steering = self.steering_save - self.coeff_dec_steering
                self.default_counter_steering = 0
                self.steering_save = 0

        if temp_steering < -math.pi / 5:
            temp_steering = -math.pi / 5

        self.steering = temp_steering

    def compute_steering(self, btn_inc_steering, btn_dec_steering):
        """
        This function computes the steering angle based on time thanks to the data frequency,
        It increments the steering when the right button is pressed and it decrements the steering if the left button is pressed
        Input :
            enum btn_inc_steering : 'PRESSED' when the right button is pressed, else 'NOT_PRESSED'
            enum btn_dec_steering : 'PRESSED' when the left button is pressed, else 'NOT_PRESSED'
        """

        if (
            btn_inc_steering != Button.PRESSED
            and btn_inc_steering != Button.NOT_PRESSED
        ) or (
            btn_dec_steering != Button.PRESSED
            and btn_dec_steering != Button.NOT_PRESSED
        ):
            return Error.InvalidValue

        if (
            btn_inc_steering == Button.PRESSED or btn_dec_steering == Button.PRESSED
        ) and abs(self.vehicle_state.vel) < self.threshold_stop:
            return Error.Speed

        # Test if left button and right button are pressed in the same time
        if btn_dec_steering == Button.PRESSED and btn_inc_steering == Button.PRESSED:
            return Error.InvalidInput

        if (
            btn_dec_steering == Button.PRESSED
            and btn_inc_steering == Button.NOT_PRESSED
            and abs(self.vehicle_state.vel) > self.threshold_stop
        ):
            self.dec_steering(btn_dec_steering)

        if (
            btn_inc_steering == Button.PRESSED
            and btn_dec_steering == Button.NOT_PRESSED
            and abs(self.vehicle_state.vel) >= self.threshold_stop
        ):
            self.inc_steering(btn_inc_steering)

        if (
            self.default_counter_steering <= self.limit_default_counter_steering
            and btn_inc_steering == Button.NOT_PRESSED
            and btn_dec_steering == Button.NOT_PRESSED
        ):
            self.default_counter_steering += 1
            if (
                self.default_counter_steering == 1
            ):  # save only the first throttle value, before the missing informations
                self.steering_save = self.steering
            self.steering = 0

        if (
            self.default_counter_steering > self.limit_default_counter_steering
            and btn_inc_steering == Button.NOT_PRESSED
            and btn_dec_steering == Button.NOT_PRESSED
        ):
            self.steering = 0
            self.default_counter_steering = 0
            self.steering_save = 0

        return Error.OK

    def autopilot(self):
        """
        This function simulates the autopilot by generating a random trajectory
        """
        self.handbrake = Handbrake.NOT_ACTIVATED

        self.random_trajectory_counter += 1

        if self.random_trajectory_counter % 100 < 15:
            btn_inc_gas = Button.PRESSED
            btn_dec_gas = Button.NOT_PRESSED
            btn_inc_steering = Button.NOT_PRESSED
            btn_dec_steering = Button.NOT_PRESSED

        elif self.random_trajectory_counter % 100 < 20:
            btn_inc_gas = Button.PRESSED
            btn_dec_gas = Button.NOT_PRESSED
            btn_inc_steering = Button.PRESSED
            btn_dec_steering = Button.NOT_PRESSED

        elif self.random_trajectory_counter % 100 < 35:
            btn_inc_gas = Button.NOT_PRESSED
            btn_dec_gas = Button.PRESSED
            btn_inc_steering = Button.PRESSED
            btn_dec_steering = Button.NOT_PRESSED

        elif self.random_trajectory_counter % 100 < 45:
            btn_inc_gas = Button.PRESSED
            btn_dec_gas = Button.NOT_PRESSED
            btn_inc_steering = Button.PRESSED
            btn_dec_steering = Button.NOT_PRESSED

        elif self.random_trajectory_counter % 100 < 50:
            btn_inc_gas = Button.PRESSED
            btn_dec_gas = Button.NOT_PRESSED
            btn_inc_steering = Button.NOT_PRESSED
            btn_dec_steering = Button.NOT_PRESSED

        elif self.random_trajectory_counter % 100 < 65:
            btn_inc_gas = Button.PRESSED
            btn_dec_gas = Button.NOT_PRESSED
            btn_inc_steering = Button.NOT_PRESSED
            btn_dec_steering = Button.PRESSED

        elif self.random_trajectory_counter % 100 < 80:
            btn_inc_gas = Button.NOT_PRESSED
            btn_dec_gas = Button.PRESSED
            btn_inc_steering = Button.NOT_PRESSED
            btn_dec_steering = Button.PRESSED

        elif self.random_trajectory_counter % 100 < 90:
            btn_inc_gas = Button.PRESSED
            btn_dec_gas = Button.NOT_PRESSED
            btn_inc_steering = Button.NOT_PRESSED
            btn_dec_steering = Button.PRESSED

        else:
            btn_inc_gas = Button.NOT_PRESSED
            btn_dec_gas = Button.NOT_PRESSED
            btn_inc_steering = Button.NOT_PRESSED
            btn_dec_steering = Button.NOT_PRESSED

        throttle_error = self.compute_throttle(btn_inc_gas, btn_dec_gas)
        steering_error = self.compute_steering(btn_inc_steering, btn_dec_steering)
        self.physic_model.simulate_step(
            self.steering, self.throttle, self.mvt_direction, self.vehicle_state
        )
        if throttle_error == Error.OK and steering_error == Error.OK:
            autopilot_error = Error.OK
        else:
            autopilot_error = Error.AutopilotError

        return autopilot_error, throttle_error, steering_error, self.vehicle_state

    def run_step(self, btn_inc_gas, btn_dec_gas, btn_inc_steering, btn_dec_steering):
        """
        This function runs the computing process of the throttle and the steering before running the Physic Model step.
        Input :
            enum btn_inc_gas : 'PRESSED' when the gas pedal is pressed, else 'NOT_PRESSED'
            enum btn_dec_gas : 'PRESSED' when the brake is pressed, else 'NOT_PRESSED'
            enum btn_inc_steering : 'PRESSED' when the right button is pressed, else 'NOT_PRESSED'
            enum btn_dec_steering : 'PRESSED' when the left button is pressed, else 'NOT_PRESSED'
        """
        if self.e_stop_state == Estop.ACTIVATED:
            if self.vehicle_state.vel >= self.threshold_stop:
                self.throttle = -1
            else:
                self.e_stop_state = Estop.NOT_ACTIVATED
                self.handbrake = Handbrake.ACTIVATED
                self.start = Starter.OFF
                return Error.EndEmergencyStop, Error.OK, Error.OK, self.vehicle_state

            self.physic_model.simulate_step(
                self.steering, self.throttle, self.mvt_direction, self.vehicle_state
            )

            return Error.OK, Error.OK, Error.OK, self.vehicle_state

        elif self.piloting == Piloting.MANUAL:
            if self.start == Starter.ON:
                throttle_error = self.compute_throttle(btn_inc_gas, btn_dec_gas)
                steering_error = self.compute_steering(
                    btn_inc_steering, btn_dec_steering
                )
                self.physic_model.simulate_step(
                    self.steering, self.throttle, self.mvt_direction, self.vehicle_state
                )
                if throttle_error == Error.OK and steering_error == Error.OK:
                    interpreter_error = Error.OK
                else:
                    interpreter_error = Error.InterpreterStepError

                return (
                    interpreter_error,
                    throttle_error,
                    steering_error,
                    self.vehicle_state,
                )

            elif self.start == Starter.OFF:
                return Error.OK, Error.OK, Error.OK, self.vehicle_state

        elif self.piloting == Piloting.AUTO:
            (
                interpreter_error,
                throttle_error,
                steering_error,
                self.vehicle_state,
            ) = self.autopilot()
            return interpreter_error, throttle_error, steering_error, self.vehicle_state


if __name__ == "__main__":
    target_frequency = 1  # in Hz
    inter = Interpreter(
        sample_time=1 / target_frequency,
        coeff_inc_throttle_sec=0.5,
        coeff_dec_throttle_sec=0.5,
        coeff_inc_steering_sec=0.3,
        coeff_dec_steering_sec=0.3,
        limit_default_counter_throttle=3,
        limit_default_counter_steering=3,
        threshold_stop=0.27,
    )
    i = 0
    n_iterations = 100 * target_frequency
    t_plot = []
