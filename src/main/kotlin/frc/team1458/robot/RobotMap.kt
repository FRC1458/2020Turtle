package frc.team1458.robot

import frc.team1458.lib.actuator.FX
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.sensor.NavX
import frc.team1458.lib.sensor.interfaces.AngleSensor


class RobotMap {
    val falcon1 = FX(canID = 1, encoderPPR = 8192.0, invert = false, invertEncoder = false)
    val falcon2 = FX(canID = 3, encoderPPR = 8192.0, invert = false, invertEncoder = false)
    val falcon3 = FX(canID = 2, encoderPPR = 8192.0, invert = false, invertEncoder = false)

    val drivetrain = Drivetrain(
            motor1 = falcon1,
            motor2 = falcon2,
            motor3 = falcon3,
            // PIDConstants(0.1,0.0,0.0,0.0711,1.17/12)
            motorOneConstants = PIDConstants.DISABLE,
            motorTwoConstants = PIDConstants.DISABLE,
            motorThreeConstants = PIDConstants.DISABLE,

            wheelDiameter = 0.51,
            kiwiRadius = 2.17,

            gyro = NavX.MXP_SPI().yaw.inverted,

            maxVoltage = 11.0
    )
    init {
        drivetrain.configPID()
    }
}