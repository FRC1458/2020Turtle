package frc.team1458.robot


import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.util.flow.systemTimeSeconds
import frc.team1458.lib.util.maths.TurtleMaths


class Robot : TimedRobot() {

    private val oi: OI = OI()
    private val robot: RobotMap = RobotMap()
    private var lockedAngle: Double = robot.drivetrain.gyro.radians
    private var isLocked: Boolean = true

    var vel = Triple(0.0, 0.0, 0.0)
    var offset: Double = 0.0
    var lastHeading: Double = 0.0
    var lastBonk: Double = 0.0
    var lastT: Double = 0.0

    override fun robotInit() {
        println("Robot Initialized")

        SmartDashboard.putNumber("bonkP", -0.002)
        SmartDashboard.putNumber("bonkD", 0.0005)

        enabledLog()
    }

    fun log() {
        SmartDashboard.putNumber("Motor1 Distance", robot.drivetrain.encoder1.distanceFeet)
        SmartDashboard.putNumber("Motor2 Distance", robot.drivetrain.encoder2.distanceFeet)
        SmartDashboard.putNumber("Motor3 Distance", robot.drivetrain.encoder3.distanceFeet)

        SmartDashboard.putNumber("Motor1 Velocity", robot.drivetrain.encoder1.velocityFeetSec)
        SmartDashboard.putNumber("Motor2 Velocity", robot.drivetrain.encoder2.velocityFeetSec)
        SmartDashboard.putNumber("Motor3 Velocity", robot.drivetrain.encoder3.velocityFeetSec)

        SmartDashboard.putNumber("NavX Theta", robot.drivetrain.gyro.radians)
        SmartDashboard.putBoolean("Is Locked", isLocked)
        SmartDashboard.putNumber("Lock Angle", lockedAngle)

        SmartDashboard.putNumber("Forward Axis", oi.forwardAxis.value)
        SmartDashboard.putNumber("Strafe Axis", oi.strafeAxis.value)
        SmartDashboard.putNumber("Rotate Axis", oi.rotateAxis.value)
    }

    fun enabledLog() {
        log()
    }

    override fun autonomousInit() {
        autonomousPeriodic()
    }

    override fun autonomousPeriodic() {
        delay(500)

        robot.falcon1.setRaw(-0.125)
        robot.falcon2.setRaw(-0.125 * 1.25)
        robot.falcon3.setRaw(0.250)

        SmartDashboard.putNumber("Motor1 VelocityDes", -0.125)
        SmartDashboard.putNumber("Motor2 VelocityDes", -0.125 * 1.25)
        SmartDashboard.putNumber("Motor3 VelocityDes", 0.250)

        enabledLog()
    }

    override fun teleopInit() {
        robot.drivetrain.zeroEncoders()
        robot.drivetrain.gyro.zero()

        lockedAngle = robot.drivetrain.gyro.radians
        lastHeading = robot.drivetrain.gyro.radians

        offset = robot.drivetrain.gyro.heading - lastHeading
    }


    override fun teleopPeriodic() {
        if (oi.frameLockButton.triggered) {
            isLocked = !isLocked
            lockedAngle = robot.drivetrain.gyro.radians
            lastHeading = robot.drivetrain.gyro.radians
            offset = 0.0
        }

        if (!isLocked) {
            vel = TurtleMaths.kiwiDrive(TurtleMaths.deadband(oi.forwardAxis.value, 0.05),
                    TurtleMaths.deadband(oi.strafeAxis.value, 0.05),
                    TurtleMaths.deadband(oi.rotateAxis.value, 0.05))

            SmartDashboard.putNumber("Motor1 VelocityDes", vel.first)
            SmartDashboard.putNumber("Motor2 VelocityDes", vel.second)
            SmartDashboard.putNumber("Motor3 VelocityDes", vel.third)

            robot.drivetrain.driveVoltageScaled(vel.first, vel.second, vel.third)

        } else if (isLocked) {
            vel = TurtleMaths.kiwiLockedAngle(TurtleMaths.deadband(oi.forwardAxis.value, 0.05),
                    TurtleMaths.deadband(oi.strafeAxis.value, 0.05),
                    0.0, robot.drivetrain.gyro.radians, lockedAngle)

            offset = robot.drivetrain.gyro.heading - lastHeading
            while (offset < 0) offset += 360
            offset %= 360
            if (offset > 180) {
                offset += -360
            }

            SmartDashboard.putNumber("bonks", offset)
            val bonkoffset = offset + 0.00000
            offset = SmartDashboard.getNumber("bonkP", Double.NaN) * offset + SmartDashboard.getNumber("bonkD", Double.NaN) * ((offset - lastBonk) / (lastT - systemTimeSeconds))
            SmartDashboard.putNumber("bonks2", offset)

            lastHeading += (systemTimeSeconds - lastT) * TurtleMaths.deadband(oi.rotateAxis.value, 0.05) * 360.0
            while (lastHeading < 0) lastHeading += 360
            lastHeading %= 360.0
            SmartDashboard.putNumber("lastHeading", lastHeading)

            lastBonk = bonkoffset
            lastT = systemTimeSeconds

            //offset = Math.min(offset, 0.12)
            //offset = Math.max(offset, -0.12)
            SmartDashboard.putNumber("bonks3", offset)

            robot.drivetrain.driveVoltageScaled(vel.first + offset, vel.second + offset, vel.third + offset)
        }

        enabledLog()
    }

    override fun testInit() {
        testPeriodic()
    }

    override fun testPeriodic() {
        if (oi.motor1Button.triggered) {
            robot.falcon1.setRaw(-0.125)
        } else {
            robot.falcon1.setRaw(0.0)
        }
        if (oi.motor2Button.triggered) {
            robot.falcon2.setRaw(-0.125 * 1.30)
        } else {
            robot.falcon2.setRaw(0.0)
        }
        if (oi.motor3Button.triggered) {
            robot.falcon3.setRaw(0.250)
        } else {
            robot.falcon3.setRaw(0.0)
        }

        SmartDashboard.putNumber("Motor1 VelocityDes", -0.125)
        SmartDashboard.putNumber("Motor2 VelocityDes", -0.125 * 1.30)
        SmartDashboard.putNumber("Motor3 VelocityDes", 0.250)

        enabledLog()
    }

    override fun disabledInit() {
        disabledPeriodic()
    }

    override fun disabledPeriodic() {
        robot.drivetrain.stop()
        offset = 0.0
        enabledLog()
    }
}

