from Interpreter.Interpreter import Interpreter
from enumFile.enum_class import (
    Starter,
    Button,
    Direction,
    Handbrake,
    Error,
    Estop,
    Piloting,
)
import math

SAMPLE_TIME = 0.01
COEEF_INC_THROTTLE = 0.02
COEEF_DEC_THROTTLE = 0.3
COEEF_INC_STEERING = 0.03
COEEF_DEC_STEERING = 0.2
LIMIT_COUNTER_THROTTLE = 4
LIMIT_COUNTER_STEERING = 6
E = 10e-6


def incr_throttle(interpreter, n):
    """
    This function incrementes the throttle of the interpreter n times
    Input :
        Interpreter : the interpreter tested
        Int n : the number of times that the throttle needs to be computed
    Output:
        Interpreter : the interpreter tested after modification of its throttle
    """
    for i in range(0, n):
        interpreter.compute_throttle(Button.PRESSED, Button.NOT_PRESSED)
    return interpreter


def decr_throttle(interpreter, n):
    """
    This function decrements the throttle of the interpreter n times
    Input :
        Interpreter : the interpreter tested
        Int n : the number of times that the throttle needs to be computed
    Output:
        Interpreter : the interpreter tested after modification of its throttle
    """
    for i in range(0, n):
        interpreter.compute_throttle(Button.NOT_PRESSED, Button.PRESSED)
    return interpreter


def zero_throttle(interpreter, n):
    """
    This function simulates no button being press for the throttle of the interpreter n times
    Input :
        Interpreter : the interpreter tested
        Int n : the number of times that the throttle needs to be computed
    Output:
        Interpreter : the interpreter tested after modification of its throttle
    """
    for i in range(0, n):
        interpreter.compute_throttle(Button.NOT_PRESSED, Button.NOT_PRESSED)
    return interpreter


def incr_steering(interpreter, n):
    """
    This function decrements the steering of the interpreter n times
    Input :
        Interpreter : the interpreter tested
        Int n : the number of times that the steering needs to be computed
    Output:
        Interpreter : the interpreter tested after modification of its steering
    """
    for i in range(0, n):
        interpreter.compute_steering(Button.PRESSED, Button.NOT_PRESSED)
    return interpreter


def decr_steering(interpreter, n):
    """
    This function decrements the steering of the interpreter n times
    Input :
        Interpreter : the interpreter tested
        Int n : the number of times that the steering needs to be computed
    Output:
        Interpreter : the interpreter tested after modification of its steering
    """
    for i in range(0, n):
        interpreter.compute_steering(Button.NOT_PRESSED, Button.PRESSED)
    return interpreter


def zero_steering(interpreter, n):
    """
    This function simulates no button being press for the steering of the interpreter n times
    Input :
        Interpreter : the interpreter tested
        Int n : the number of times that the steering needs to be computed
    Output:
        Interpreter : the interpreter tested after modification of its steering
    """
    for i in range(0, n):
        interpreter.compute_steering(Button.NOT_PRESSED, Button.NOT_PRESSED)
    return interpreter


def test_computethrottle():
    """
    This function tests various cases concerning the throttle computation
    """

    inte1 = Interpreter(
        SAMPLE_TIME,
        COEEF_INC_THROTTLE,
        COEEF_DEC_THROTTLE,
        COEEF_INC_STEERING,
        COEEF_DEC_STEERING,
        LIMIT_COUNTER_THROTTLE,
        LIMIT_COUNTER_STEERING,
    )
    inte1.vehicle_state.vel = 5
    inte1.handbrake = Handbrake.NOT_ACTIVATED

    then = incr_throttle(inte1, 4)
    assert (
        abs(then.throttle - COEEF_INC_THROTTLE * SAMPLE_TIME * 4) <= E
        and then.throttle > 0
    )
    then = incr_throttle(inte1, 10000)
    assert abs(1 - then.throttle) <= E

    then = zero_throttle(inte1, 2)
    assert (
        then.throttle == 0
        and then.default_counter_throttle == 2
        and abs(1 - then.throttle_save) <= E
    )
    then = zero_throttle(inte1, LIMIT_COUNTER_THROTTLE - 2)
    assert (
        then.throttle == 0
        and then.default_counter_throttle == LIMIT_COUNTER_THROTTLE
        and abs(1 - then.throttle_save) <= E
    )
    then = zero_throttle(inte1, 1)
    assert (
        then.throttle == 0
        and then.default_counter_throttle == 0
        and then.throttle_save == 0
    )

    then = decr_throttle(inte1, 4)
    assert (
        abs(then.throttle + COEEF_DEC_THROTTLE * SAMPLE_TIME * 4) <= E
        and then.throttle < 0
    )
    then = decr_throttle(inte1, 10000)
    assert abs(-1 - then.throttle) <= E

    then = zero_throttle(inte1, 2)
    assert (
        then.throttle == 0
        and then.default_counter_throttle == 2
        and abs(-1 - then.throttle_save) <= E
    )
    then = zero_throttle(inte1, LIMIT_COUNTER_THROTTLE - 2)
    assert (
        then.throttle == 0
        and then.default_counter_throttle == LIMIT_COUNTER_THROTTLE
        and abs(-1 - then.throttle_save) <= E
    )
    then = zero_throttle(inte1, 1)
    assert (
        then.throttle == 0
        and then.default_counter_throttle == 0
        and then.throttle_save == 0
    )

    then = incr_throttle(inte1, 3)
    assert (
        abs(then.throttle - COEEF_INC_THROTTLE * SAMPLE_TIME * 3) <= E
        and then.throttle > 0
    )
    then = zero_throttle(inte1, LIMIT_COUNTER_THROTTLE - 2)
    assert (
        then.throttle == 0
        and then.default_counter_throttle == LIMIT_COUNTER_THROTTLE - 2
        and abs(then.throttle_save - COEEF_INC_THROTTLE * SAMPLE_TIME * 3) <= E
        and then.throttle_save > 0
    )

    then = decr_throttle(inte1, 5)
    assert (
        abs(then.throttle + COEEF_DEC_THROTTLE * SAMPLE_TIME * 5) <= E
        and then.throttle < 0
    )

    then = zero_throttle(inte1, LIMIT_COUNTER_THROTTLE - 1)
    assert (
        then.throttle == 0
        and then.default_counter_throttle == LIMIT_COUNTER_THROTTLE - 1
        and abs(then.throttle_save + COEEF_DEC_THROTTLE * SAMPLE_TIME * 5) <= E
    )
    then = incr_throttle(inte1, 6)
    assert (
        abs(then.throttle - COEEF_INC_THROTTLE * SAMPLE_TIME * 6) <= E
        and then.throttle > 0
    )


def test_computesteering():
    """
    This function tests various cases concerning the steering computation
    """

    inte1 = Interpreter(
        SAMPLE_TIME,
        COEEF_INC_THROTTLE,
        COEEF_DEC_THROTTLE,
        COEEF_INC_STEERING,
        COEEF_DEC_STEERING,
        LIMIT_COUNTER_THROTTLE,
        LIMIT_COUNTER_STEERING,
    )
    inte1.vehicle_state.vel = 5

    then = incr_steering(inte1, 4)
    assert (
        abs(then.steering - COEEF_INC_STEERING * SAMPLE_TIME * 4) <= E
        and then.steering > 0
    )
    then = incr_steering(inte1, 10000)
    assert abs(math.pi / 5 - then.steering) <= E

    then = zero_steering(inte1, 2)
    assert (
        then.steering == 0
        and then.default_counter_steering == 2
        and abs(math.pi / 5 - then.steering_save) <= E
    )
    then = zero_steering(inte1, LIMIT_COUNTER_STEERING - 2)
    assert (
        then.steering == 0
        and then.default_counter_steering == LIMIT_COUNTER_STEERING
        and abs(math.pi / 5 - then.steering_save) <= E
    )
    then = zero_steering(inte1, 1)
    assert (
        then.steering == 0
        and then.default_counter_steering == 0
        and then.steering_save == 0
    )

    then = decr_steering(inte1, 4)
    assert (
        abs(then.steering + COEEF_DEC_STEERING * SAMPLE_TIME * 4) <= E
        and then.steering < 0
    )
    then = decr_steering(inte1, 10000)
    assert abs(-math.pi / 5 - then.steering) <= E

    then = zero_steering(inte1, 2)
    assert (
        then.steering == 0
        and then.default_counter_steering == 2
        and abs(-math.pi / 5 - then.steering_save) <= E
    )
    then = zero_steering(inte1, LIMIT_COUNTER_STEERING - 2)
    assert (
        then.steering == 0
        and then.default_counter_steering == LIMIT_COUNTER_STEERING
        and abs(-math.pi / 5 - then.steering_save) <= E
    )
    then = zero_steering(inte1, 1)
    assert (
        then.steering == 0
        and then.default_counter_steering == 0
        and then.steering_save == 0
    )

    then = incr_steering(inte1, 3)
    assert (
        abs(then.steering - COEEF_INC_STEERING * SAMPLE_TIME * 3) <= E
        and then.steering > 0
    )
    then = zero_steering(inte1, LIMIT_COUNTER_STEERING - 2)
    assert (
        then.steering == 0
        and then.default_counter_steering == LIMIT_COUNTER_STEERING - 2
        and abs(then.steering_save - COEEF_INC_STEERING * SAMPLE_TIME * 3) <= E
    )

    then = decr_steering(inte1, 5)
    assert (
        abs(
            then.steering
            - (
                COEEF_INC_STEERING * SAMPLE_TIME * 3
                - COEEF_DEC_STEERING * SAMPLE_TIME * 5
            )
        )
        <= E
    )
    then = zero_steering(inte1, LIMIT_COUNTER_STEERING - 1)
    assert (
        then.steering == 0
        and then.default_counter_steering == LIMIT_COUNTER_STEERING - 1
        and abs(
            then.steering_save
            - (
                COEEF_INC_STEERING * SAMPLE_TIME * 3
                - COEEF_DEC_STEERING * SAMPLE_TIME * 5
            )
        )
        <= E
    )
    then = incr_steering(inte1, 6)
    assert (
        abs(
            then.steering
            - (
                COEEF_INC_STEERING * SAMPLE_TIME * 3
                - COEEF_DEC_STEERING * SAMPLE_TIME * 5
                + COEEF_INC_STEERING * SAMPLE_TIME * 6
            )
        )
        <= E
    )


def test_error():
    """
    This function tests various cases concerning the error that must be raised
    """
    inte1 = Interpreter(
        SAMPLE_TIME,
        COEEF_INC_THROTTLE,
        COEEF_DEC_THROTTLE,
        COEEF_INC_STEERING,
        COEEF_DEC_STEERING,
        LIMIT_COUNTER_THROTTLE,
        LIMIT_COUNTER_STEERING,
    )
    inte1.set_handbrake(Handbrake.NOT_ACTIVATED)
    inte1.vehicle_state.vel = 0.3

    assert inte1.compute_steering(Button.NOT_PRESSED, "invalid") == Error.InvalidValue
    assert inte1.compute_steering(Button.PRESSED, "invalid") == Error.InvalidValue
    assert inte1.compute_steering("invalid", Button.NOT_PRESSED) == Error.InvalidValue
    assert inte1.compute_steering("invalid", Button.PRESSED) == Error.InvalidValue
    assert inte1.compute_steering(Button.PRESSED, Button.PRESSED) == Error.InvalidInput

    inte1.vehicle_state.vel = 0
    assert inte1.compute_steering(Button.NOT_PRESSED, Button.PRESSED) == Error.Speed
    assert inte1.compute_steering(Button.PRESSED, Button.NOT_PRESSED) == Error.Speed

    assert inte1.compute_throttle(Button.NOT_PRESSED, "invalid") == Error.InvalidValue
    assert inte1.compute_throttle(Button.PRESSED, "invalid") == Error.InvalidValue
    assert inte1.compute_throttle("invalid", Button.NOT_PRESSED) == Error.InvalidValue
    assert inte1.compute_throttle("invalid", Button.PRESSED) == Error.InvalidValue
    assert inte1.compute_throttle(Button.PRESSED, Button.PRESSED) == Error.InvalidInput

    inte1.set_handbrake(Handbrake.ACTIVATED)
    assert (
        inte1.compute_throttle(Button.PRESSED, Button.NOT_PRESSED) == Error.BrakeErrorOn
    )
    assert inte1.compute_steering(Button.PRESSED, Button.NOT_PRESSED) == Error.Speed
    inte1.set_handbrake(Handbrake.NOT_ACTIVATED)
    assert (
        inte1.compute_throttle(Button.PRESSED, Button.NOT_PRESSED) != Error.BrakeErrorOn
    )


def test_set():
    """
    This function tests that the enum are set in the right state
    """

    inte1 = Interpreter(
        SAMPLE_TIME,
        COEEF_INC_THROTTLE,
        COEEF_DEC_THROTTLE,
        COEEF_INC_STEERING,
        COEEF_DEC_STEERING,
        LIMIT_COUNTER_THROTTLE,
        LIMIT_COUNTER_STEERING,
    )
    inte1.vehicle_state.vel = 0

    assert inte1.e_stop_state == Estop.NOT_ACTIVATED

    assert inte1.set_start(Starter.ON) == Error.HelpAutopilot
    inte1.piloting = Piloting.MANUAL
    assert inte1.set_start(Starter.ON) == Error.OK
    assert inte1.start == Starter.ON
    assert inte1.set_start(Starter.OFF) == Error.OK
    assert inte1.start == Starter.OFF
    assert inte1.set_start("invalid") == Error.InvalidValue

    assert inte1.set_handbrake(Handbrake.ACTIVATED) == Error.OK
    assert inte1.handbrake == Handbrake.ACTIVATED
    assert inte1.set_handbrake(Handbrake.NOT_ACTIVATED) == Error.OK
    assert inte1.handbrake == Handbrake.NOT_ACTIVATED
    assert inte1.set_handbrake("invalid") == Error.InvalidValue

    assert inte1.set_mvt_direction(Direction.FORWARD) == Error.OK
    assert inte1.mvt_direction == Direction.FORWARD
    assert inte1.set_mvt_direction(Direction.BACKWARD) == Error.OK
    assert inte1.mvt_direction == Direction.BACKWARD
    assert inte1.set_mvt_direction(Direction.FORWARD) == Error.OK
    assert inte1.mvt_direction == Direction.FORWARD
    assert inte1.set_mvt_direction("invalid") == Error.InvalidValue

    inte1.vehicle_state.vel = 2

    assert inte1.set_start(Starter.ON) == Error.OK
    assert inte1.start == Starter.ON
    assert inte1.set_start(Starter.OFF) == Error.OK
    assert inte1.e_stop_state == Estop.ACTIVATED

    assert inte1.set_handbrake(Handbrake.NOT_ACTIVATED) == Error.OK
    assert inte1.handbrake == Handbrake.NOT_ACTIVATED
    assert inte1.set_handbrake(Handbrake.ACTIVATED) == Error.BrakeErrorOff
    assert inte1.handbrake == Handbrake.NOT_ACTIVATED

    assert inte1.set_mvt_direction(Direction.FORWARD) == Error.OK
    assert inte1.mvt_direction == Direction.FORWARD
    assert inte1.set_mvt_direction(Direction.BACKWARD) == Error.DirectionError
    assert inte1.mvt_direction == Direction.FORWARD

    assert inte1.set_piloting("invalid") == Error.InvalidValue
    assert inte1.set_piloting(Piloting.AUTO) == Error.SpeedAutopilot
    inte1.vehicle_state.vel = 0
    assert inte1.set_piloting(Piloting.AUTO) == Error.OK
    assert inte1.piloting == Piloting.AUTO
    inte1.e_stop_state = Estop.NOT_ACTIVATED
    assert inte1.set_piloting(Piloting.MANUAL) == Error.HelpAutopilot
    inte1.e_stop_state = Estop.ACTIVATED
    assert inte1.set_piloting(Piloting.MANUAL) == Error.OK
    assert inte1.piloting == Piloting.MANUAL


def test_runstep():
    """
    This function tests the run step
    """
    inte1 = Interpreter(
        SAMPLE_TIME,
        COEEF_INC_THROTTLE,
        COEEF_DEC_THROTTLE,
        COEEF_INC_STEERING,
        COEEF_DEC_STEERING,
        LIMIT_COUNTER_THROTTLE,
        LIMIT_COUNTER_STEERING,
    )
    inte1.piloting = Piloting.MANUAL
    # initialization
    assert inte1.start == Starter.OFF
    assert inte1.throttle == 0
    assert inte1.steering == 0
    assert inte1.default_counter_throttle == 0
    assert inte1.default_counter_steering == 0
    assert inte1.throttle_save == 0
    assert inte1.steering_save == 0
    assert inte1.handbrake == Handbrake.ACTIVATED
    assert inte1.e_stop_state == Estop.NOT_ACTIVATED
    assert inte1.mvt_direction == Direction.FORWARD

    inte1.run_step(
        Button.NOT_PRESSED,
        Button.NOT_PRESSED,
        Button.NOT_PRESSED,
        Button.NOT_PRESSED,
    )

    # nothing has changed
    assert inte1.start == Starter.OFF
    assert inte1.throttle == 0
    assert inte1.steering == 0
    assert inte1.default_counter_throttle == 0
    assert inte1.default_counter_steering == 0
    assert inte1.throttle_save == 0
    assert inte1.steering_save == 0
    assert inte1.handbrake == Handbrake.ACTIVATED
    assert inte1.e_stop_state == Estop.NOT_ACTIVATED
    assert inte1.mvt_direction == Direction.FORWARD

    inte1.set_start(Starter.ON)

    # enters compute_throttle and compute_steering

    inte1.run_step(
        Button.NOT_PRESSED,
        Button.NOT_PRESSED,
        Button.NOT_PRESSED,
        Button.NOT_PRESSED,
    )
    assert inte1.default_counter_throttle == 1
    assert inte1.default_counter_steering == 1

    # activation of the emergency stop
    inte1.vehicle_state.vel = 2
    assert inte1.set_start(Starter.OFF) == Error.OK
    assert inte1.e_stop_state == Estop.ACTIVATED

    inte1.run_step(
        Button.NOT_PRESSED,
        Button.NOT_PRESSED,
        Button.NOT_PRESSED,
        Button.NOT_PRESSED,
    )
    assert inte1.throttle == -1

    # end of the emergency stop
    inte1.vehicle_state.vel = 0

    inte1.run_step(
        Button.NOT_PRESSED,
        Button.NOT_PRESSED,
        Button.NOT_PRESSED,
        Button.NOT_PRESSED,
    )
    assert inte1.e_stop_state == Estop.NOT_ACTIVATED
    assert inte1.handbrake == Handbrake.ACTIVATED
    assert inte1.start == Starter.OFF
