package frc.team1458.robot

import edu.wpi.first.wpilibj.XboxController
import frc.team1458.lib.input.FlightStick
import frc.team1458.lib.input.Gamepad
import frc.team1458.lib.input.interfaces.POV
import frc.team1458.lib.input.interfaces.Switch

class OI {
    // 1: Forward/Back, 0: Left/Right Strafe, 2: Rotate
    /*
    val xboxController: Gamepad = Gamepad.xboxController(0)

    var throttle = xboxController.leftY.scale(0.7).inverted
    var steer = xboxController.rightX.scale(0.5)
    var turn
     */
    private val flightJoystick: Gamepad = Gamepad.xboxController(0)

    val forwardAxis = flightJoystick.getAxis(0).scale(1.0) // 0.18
    val strafeAxis = flightJoystick.getAxis(1).scale(1.0) // 0.18
    val rotateAxis = flightJoystick.getAxis(4).scale(0.25).inverted // 2

    val frameLockButton = flightJoystick.getButton(1) // A button
    val followTurn = flightJoystick.getButton(3) // X button

    val motor1Button = flightJoystick.getButton(3) // X button
    val motor2Button = flightJoystick.getButton(1) // A button
    val motor3Button = flightJoystick.getButton(2) // B button



}
