package frc.team1458.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.Trajectory
import frc.team1458.lib.actuator.FX
import frc.team1458.lib.actuator.SRX
import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.actuator.Solenoid
import frc.team1458.lib.pathing.PathGenerator
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.sensor.interfaces.AngleSensor
import frc.team1458.lib.sensor.interfaces.DistanceSensor
import frc.team1458.lib.util.LiveDashboard
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.util.flow.systemTimeMillis
import frc.team1458.lib.util.flow.systemTimeSeconds
import frc.team1458.lib.util.maths.TurtleMaths
import java.lang.Math.pow
import kotlin.math.IEEErem
import kotlin.math.abs

// Notice: this code will be jank - this class should wrap as much of the jank behavior as possible,
// in order to make the interfaces for other classes easier


class Drivetrain(val motor1: FX,
                 val motor2: FX,
                 val motor3: FX,

                 val wheelDiameter: Double,
                 val kiwiRadius: Double,
                 var lockedFrame: Boolean = false,

                 val angularAcc: Double = 0.1,

                 val gyro: AngleSensor,
                 val invertGyro: Boolean = false,

                 val maxVoltage: Double = 11.0) {

    val wheelCircumference = wheelDiameter.times(Math.PI)

    val encoder1: DistanceSensor = object : DistanceSensor {
        override val distanceMeters: Double
            get() = (motor1.encoder.angle * (wheelCircumference ?: 0.0) * 0.3048 / 360.0)

        override val velocity: Double
            get() = motor1.encoder.rate * (wheelCircumference ?: 0.0) * 0.3048 / 360.0

        override fun zero() {
            motor1.encoder.zero()
        }
    }

    val encoder2: DistanceSensor = object : DistanceSensor {
        override val distanceMeters: Double
            get() = (motor2.encoder.angle * (wheelCircumference ?: 0.0) * 0.3048 / 360.0)

        override val velocity: Double
            get() = motor2.encoder.rate * (wheelCircumference ?: 0.0) * 0.3048 / 360.0

        override fun zero() {
            motor2.encoder.zero()
        }
    }

    val encoder3: DistanceSensor = object : DistanceSensor {
        override val distanceMeters: Double
            get() = (motor3.encoder.angle * (wheelCircumference ?: 0.0) * 0.3048 / 360.0)

        override val velocity: Double
            get() = motor3.encoder.rate * (wheelCircumference ?: 0.0) * 0.3048 / 360.0

        override fun zero() {
            motor3.encoder.zero()
        }
    }

    // Use this when driving from a controller and your inputs are (-1.0, 1.0)
    fun driveVoltageScaled(motor1Control: Double, motor2Control: Double, motor3Control: Double) {
        motor1.setVoltage(maxVoltage * motor1Control)
        motor2.setVoltage(maxVoltage * motor2Control)
        motor3.setVoltage(maxVoltage * motor3Control)
    }

    fun driveVoltage(motor1Control: Double, motor2Control: Double, motor3Control: Double) {
        motor1.setVoltage(motor1Control)
        motor2.setVoltage(motor2Control)
        motor3.setVoltage(motor3Control)
    }

    fun driveVelocity(motor1Control: Double, motor2Control: Double, motor3Control: Double) {
        // Convert from ft/sec to deg/sec
        motor1.setVelocity(motor1Control / ((wheelCircumference ?: 0.0) / 360.0))
        motor2.setVelocity(motor2Control / ((wheelCircumference ?: 0.0) / 360.0))
        motor3.setVelocity(motor3Control / ((wheelCircumference ?: 0.0) / 360.0))
    }

    // linvel = ft/sec, angvel = rad/sec
    fun driveCmdVel(linvel: Double, angvel: Double) {
    }

    fun stop() {
        driveVoltage(0.0, 0.0, 0.0)
    }
    /*
    fun clearOdom(clearGyro: Boolean = true, clearEncs: Boolean = true) {
        try {
            if (clearGyro) {
                gyro.zero()
            }

            if (clearEncs) {
                leftEnc.zero()
                rightEnc.zero()
            }

            odom.resetPosition(Pose2d(), Rotation2d.fromDegrees(0.0))
            odomTrust = 1.0
        } catch (e: Exception) {
            odomTrust = -1.0

            println("WARNING - clearOdom failed! Most likely disconnected gyro/encoders")
            // e.printStackTrace()
        } finally {
        }
    }

    // Update odometry - call this at the end of teleopPeriodic
    fun updateOdom(dashboard: Boolean = true) {
        fun getHeading() = gyro.angle.IEEErem(360.0) * (if (invertGyro) {
            -1.0
        } else {
            1.0
        })

        fun getHeadingRate() = gyro.rate * (if (invertGyro) {
            -1.0
        } else {
            1.0
        })

        try {
            odom.update(Rotation2d.fromDegrees(getHeading()), leftEnc.distanceFeet, rightEnc.distanceFeet)

            // TODO although this says "poseMeters", we are actually using feet for our measurements
            if (dashboard) {
                LiveDashboard.putOdom(pose.translation.x, pose.translation.y, pose.rotation.radians)
                LiveDashboard.putPath(pose.translation.x, pose.translation.y, pose.rotation.radians)
            }

            if (odomTrust != -1.0) {
                odomTrust = TurtleMaths.constrain(max = 1.0, min = 0.0, value = odomTrust + 0.05)
            }
        } catch (e: Exception) {
            if (odomTrust != -1.0) {
                odomTrust = TurtleMaths.constrain(max = 1.0, min = 0.0, value = odomTrust - 0.20)
            }

            println("WARNING - updateOdom failed! Most likely disconnected gyro/encoders")
            // e.printStackTrace()
        } finally {
        }
    }
    */
}