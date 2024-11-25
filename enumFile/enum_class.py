from enum import Enum


class Button(Enum):
    PRESSED = 1
    NOT_PRESSED = 0


class Starter(Enum):
    ON = 1
    OFF = 2


class Direction(Enum):
    FORWARD = 1
    BACKWARD = 0


class Handbrake(Enum):
    ACTIVATED = 1
    NOT_ACTIVATED = 0


class Estop(Enum):
    ACTIVATED = 1
    NOT_ACTIVATED = 0


class Piloting(Enum):
    AUTO = 0
    MANUAL = 1


class Error(Enum):
    OK = 0  # a commenter si besoin
    InvalidValue = 1  # erreur de valeur
    BrakeErrorOff = 2  # vitesse trop rapide pour l'activer
    BrakeErrorOn = 3  # il faut le desactiver pour accelerer
    DirectionError = 4  # vitesse trop rapide pour changer direction
    InvalidInput = 5  # plusieurs boutons appuyés en simultanés
    Speed = 6  # il faut accelerer pour tourner
    InterpreterStepError = 7
    EndEmergencyStop = 8
    NoFileName = 9
    AutopilotError = 10
    HelpAutopilot = 11
    SpeedAutopilot = 12
