package frc.team1458.robot

import frc.team1458.lib.actuator.FX
import frc.team1458.lib.sensor.NavX
import frc.team1458.lib.sensor.interfaces.AngleSensor


class RobotMap {
    val falcon1 = FX(canID = 1, encoderPPR = 18000.0, invert = false, invertEncoder = false)
    val falcon2 = FX(canID = 2, encoderPPR = 18000.0, invert = false, invertEncoder = false)
    val falcon3 = FX(canID = 3, encoderPPR = 18000.0, invert = false, invertEncoder = false)

    val drivetrain = Drivetrain(
            motor1 = falcon1,
            motor2 = falcon2,
            motor3 = falcon3,

            wheelDiameter = 0.51,
            kiwiRadius = 2.17,

            gyro = NavX.MXP_I2C().yaw,
            invertGyro = true,

            maxVoltage = 11.0
    )
}