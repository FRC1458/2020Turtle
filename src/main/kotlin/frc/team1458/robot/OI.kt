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

    var forwardAxis = flightJoystick.getAxis(1).scale(0.25)
    var strafeAxis = flightJoystick.getAxis(0).scale(0.25)
    var rotateAxis = flightJoystick.getAxis(2).scale(0.25)



}
