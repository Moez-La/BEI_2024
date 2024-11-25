from Model.model_vehicle import *
from enumFile.enum_class import *


# Test if the limit of velocity is working
def test_saturation():
    # Create simulation of 20 seconds, with sample time of 0.01
    simTime = 20.0
    ts = 0.01

    model = PhysicsModel(ts=ts)
    steer = 0
    throttle = 1
    vehicle_state = VehicleState()

    # Accelerates completely during a period of time long enough to achieve saturation
    for i in range(int(simTime / ts)):
        model.simulate_step(
            steer=steer,
            throttle=throttle,
            direction=Direction.FORWARD,
            vehicle_state=vehicle_state,
        )

        # The velocity needs to be saturated
        assert vehicle_state.vel <= 20


# Test function for braking forward
def test_braking_forward():
    # Create simulation of 20 seconds, with sample time of 0.01
    simTime = 20.0
    ts = 0.01

    threshold_stop = 0.05
    model = PhysicsModel(ts=ts)
    steer = 0
    vehicle_state = VehicleState()

    # Accelerates until reach a high velocity
    for i in range(int(simTime / ts)):
        model.simulate_step(
            steer=steer,
            throttle=1,
            direction=Direction.FORWARD,
            vehicle_state=vehicle_state,
        )

    # Brake until stops the vehicle
    for i in range(int(simTime / ts)):
        model.simulate_step(
            steer=steer,
            throttle=-1,
            direction=Direction.FORWARD,
            vehicle_state=vehicle_state,
        )

    # Check if final velocity is close to zero
    assert vehicle_state.vel < threshold_stop


#  test function for braking backward
def test_braking_backward():
    # Create simulation of 20 seconds, with sample time of 0.01
    simTime = 20.0
    ts = 0.01

    threshold_stop = 0.05
    model = PhysicsModel(ts=ts)
    steer = 0
    vehicle_state = VehicleState()

    # Accelerates until reach a high velocity
    for i in range(int(simTime / ts)):
        model.simulate_step(
            steer=steer,
            throttle=1,
            direction=Direction.FORWARD,
            vehicle_state=vehicle_state,
        )

    # Brake until stops the vehicle
    for i in range(int(simTime / ts)):
        model.simulate_step(
            steer=steer,
            throttle=-1,
            direction=Direction.FORWARD,
            vehicle_state=vehicle_state,
        )

    # Check if final velocity is close to zero
    assert vehicle_state.vel < threshold_stop


#  test function for partial braking
def test_partial_braking():
    # Create simulation of 20 seconds, with sample time of 0.01
    simTime = 20.0
    ts = 0.01

    threshold_velocity = 5
    model = PhysicsModel(ts=ts)
    steer = 0
    throttle = -0.5
    vehicle_state = VehicleState()

    # Brake the vehicle partially
    for i in range(int(simTime / ts)):
        model.simulate_step(
            steer=steer,
            throttle=throttle,
            direction=Direction.FORWARD,
            vehicle_state=vehicle_state,
        )

    # Check if final velocity is below the threshold (indicating partial braking effect)
    assert vehicle_state.vel < threshold_velocity


# test if the velocity increases in the FORWARD mode
def test_forward_mode():
    ts = 0.01
    model = PhysicsModel(ts=ts)
    steer = 0
    throttle = 1
    vehicle_state = VehicleState()

    # Accelerates the vehicle
    model.simulate_step(
        steer=steer,
        throttle=throttle,
        direction=Direction.FORWARD,
        vehicle_state=vehicle_state,
    )

    # Check if velocity increases
    assert vehicle_state.vel > 0


# test if the velocity decreases in the BACKWARD mode
def test_backward_mode():
    ts = 0.01
    model = PhysicsModel(ts=ts)
    steer = 0
    throttle = 1
    vehicle_state = VehicleState()

    # Accelerates the vehicle
    model.simulate_step(
        steer=steer,
        throttle=throttle,
        direction=Direction.BACKWARD,
        vehicle_state=vehicle_state,
    )

    # Check if velocity decreases
    assert vehicle_state.vel < 0


# test of velocity for many steps
def test_many_steps():
    ts = 0.01

    model = PhysicsModel(ts=ts)

    vehicle_state = VehicleState()

    # Accelerates the vehicle
    for i in range(500):
        prev_vel = vehicle_state.vel
        model.simulate_step(
            steer=0,
            throttle=1,
            direction=Direction.FORWARD,
            vehicle_state=vehicle_state,
        )
        assert vehicle_state.vel >= prev_vel
    for i in range(500):
        prev_vel = vehicle_state.vel
        model.simulate_step(
            steer=0,
            throttle=0,
            direction=Direction.FORWARD,
            vehicle_state=vehicle_state,
        )
        assert vehicle_state.vel <= prev_vel
    for i in range(500):
        prev_vel = vehicle_state.vel
        model.simulate_step(
            steer=0,
            throttle=-1,
            direction=Direction.FORWARD,
            vehicle_state=vehicle_state,
        )
        assert vehicle_state.vel <= prev_vel
    for i in range(500):
        prev_theta = vehicle_state.theta
        model.simulate_step(
            steer=math.pi / 5,
            throttle=0,
            direction=Direction.FORWARD,
            vehicle_state=vehicle_state,
        )
        assert vehicle_state.theta >= prev_theta
    for i in range(500):
        prev_theta = vehicle_state.theta
        model.simulate_step(
            steer=-math.pi / 5,
            throttle=0,
            direction=Direction.FORWARD,
            vehicle_state=vehicle_state,
        )
        assert vehicle_state.theta <= prev_theta
