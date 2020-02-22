package frc.team1458.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team1458.lib.actuator.SRX
import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.input.Gamepad
import frc.team1458.lib.pathing.PathGenerator
import frc.team1458.lib.util.LiveDashboard
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.util.flow.systemTimeSeconds
import frc.team1458.lib.util.maths.TurtleMaths


class Robot : TimedRobot() {

    private val oi: OI = OI()
    private val robot: RobotMap = RobotMap()

    override fun robotInit() {
        println("Robot Initialized")
    }

    fun log() {
        /*
        SmartDashboard.putNumber("Left Distance", robot.drivetrain.leftEnc.distanceFeet)
        SmartDashboard.putNumber("Right Distance", robot.drivetrain.rightEnc.distanceFeet)

        SmartDashboard.putNumber("Left Velocity", robot.drivetrain.leftEnc.velocityFeetSec)
        SmartDashboard.putNumber("Right Velocity", robot.drivetrain.rightEnc.velocityFeetSec)

        SmartDashboard.putNumber("Left Error", robot.drivetrain.leftClosedLoopError)
        SmartDashboard.putNumber("Right Error", robot.drivetrain.rightClosedLoopError)

        SmartDashboard.putNumber("Odom X Feet", robot.drivetrain.pose.translation.x)
        SmartDashboard.putNumber("Odom Y Feet", robot.drivetrain.pose.translation.y)
        SmartDashboard.putNumber("Odom Theta Deg", robot.drivetrain.pose.rotation.degrees)

        SmartDashboard.putNumber("NavX Theta", robot.drivetrain.gyro.angle)
         */
    }

    fun enabledLog() {
        log()
    }

    override fun autonomousInit() {
    }

    override fun autonomousPeriodic() {
        disabledInit()
    }

    override fun teleopInit() {
        teleopPeriodic()
    }

    override fun teleopPeriodic() {
        val (v1, v2, v3) = TurtleMaths.kiwiDrive(TurtleMaths.deadband(oi.forwardAxis.value, 0.05),
                                                 TurtleMaths.deadband(oi.strafeAxis.value, 0.05),
                                                 TurtleMaths.deadband(oi.rotateAxis.value, 0.05))

        robot.drivetrain.driveVoltageScaled(v1, v2, v3)

        enabledLog()
    }

    override fun testInit() {
    }

    override fun testPeriodic() {
    }

    override fun disabledInit() {
        robot.drivetrain.stop()
    }

    override fun disabledPeriodic() {
        robot.drivetrain.stop()
        log()
    }
}

