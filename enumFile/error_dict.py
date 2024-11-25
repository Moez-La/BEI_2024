from enumFile.enum_class import Error

error_dict = {
    Error.InvalidValue: "The input value is incorrect",  # InvalidValue
    Error.BrakeErrorOff: "Please slow down before activating the handbrake",  # BrakeErrorOff
    Error.BrakeErrorOn: "Please desactivate the handbrake before accelerating",  # BrakeErrorOn
    Error.DirectionError: "Please slow down before changing direction",  # DirectionError
    Error.InvalidInput: "Please press only one button",  # InvalidInput
    Error.Speed: "Please increase the speed before turning",  # Speed
    Error.InterpreterStepError: "An error has occured in the interpreter's run step",  # InterpreterStepError
    Error.EndEmergencyStop: "Emergency stop has been successfully ended",
    Error.NoFileName: "Please enter a file name before saving",
    Error.AutopilotError: "An error has occured in the autopilot mode",
    Error.HelpAutopilot: "The autopilot does not need help",
    Error.SpeedAutopilot: "Please slow down before activating autopilot",
}
