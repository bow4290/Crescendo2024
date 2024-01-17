package frc.robot;

import static edu.wpi.first.wpilibj.PS4Controller.Button.*;
import static edu.wpi.first.wpilibj.XboxController.Button.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Handles input from Xbox/Logitech or PS4/PS5 controllers connected to the Driver Station. Also has
 * code for handling Logitech F310 gamepad in DirectInput mode on simulation. (The Logitech gamepads
 * have a switch on the back, D/X, to switch between DirectInput and Xbox mode. The gamepad should
 * be set to X whenever connected to the Driver Station.)
 */
@SuppressWarnings(
    "unused") // otherwise intellij complains unless every single thingy is used somewhere
public class GenericGamepad { // todo: rename to GenericGamepad after testing it
  public enum ControllerType {
    Unconnected,
    PS4,
    Xbox,
    Logi
  }

  public static final ControllerType defaultType =
      ControllerType.PS4; // hopefully autodetect code works, though!

  public int port;
  public ControllerType currentType;

  /** Equivalent to × Blue Cross on PS4. Bottom button. */
  public Trigger a_cross = button(kCross, kA, 2);
  /** Equivalent to ○ Red Circle on PS4. Right button. */
  public Trigger b_circle = button(kCircle, kB, 3);
  /** Equivalent to □ Purple Square on PS4. Left button. */
  public Trigger x_square = button(kSquare, kX, 1);
  /** Equivalent to △ Green Triangle on PS4. Top button. */
  public Trigger y_triangle = button(kTriangle, kY, 4);
  /** Equivalent to L1 on PS4 */
  public Trigger leftBumper = button(kL1, kLeftBumper, 5);
  /** Equivalent to R1 on PS4 */
  public Trigger rightBumper = button(kR1, kRightBumper, 6);

  public Trigger leftJoystickPushed = button(kL3, kLeftStick, 11);
  public Trigger rightJoystickPushed = button(kR3, kRightStick, 12);

  /** Equivalent to Back on Xbox or Share on PS4 */
  public Trigger leftMiddle = button(kShare, kBack, 9);
  /** Equivalent to Start on Xbox or Options on PS4 */
  public Trigger rightMiddle = button(kOptions, kStart, 10);
  /** Equivalent to Touchpad on PS4 - No equivalent on xbox */
  public Trigger topMiddle = button(kTouchpad, null, null);
  /** Equivalent to Options on PS4 - No equivalent on xbox */
  public Trigger bottomMiddle = button(kOptions, null, null);

  public DoubleSupplier leftTrigger =
      axis(PS4Controller.Axis.kL2, XboxController.Axis.kLeftTrigger, null);
  public DoubleSupplier rightTrigger =
      axis(PS4Controller.Axis.kL2, XboxController.Axis.kLeftTrigger, null);
  /** Note: joystick y values are -1 when fully pushed up * */
  public DoubleSupplier leftY = axis(PS4Controller.Axis.kLeftY, XboxController.Axis.kLeftY, 1);

  public DoubleSupplier leftX = axis(PS4Controller.Axis.kLeftX, XboxController.Axis.kLeftX, 0);
  /** Note: joystick y values are -1 when fully pushed up * */
  public DoubleSupplier rightY = axis(PS4Controller.Axis.kRightY, XboxController.Axis.kRightY, 3);

  public DoubleSupplier rightX = axis(PS4Controller.Axis.kRightX, XboxController.Axis.kRightX, 2);

  public GenericGamepad(int port) {
    this.port = port;
  }

  // backwards compatibility
  @Deprecated
  public static GenericGamepad from(int port) {
    return new GenericGamepad(port);
  }

  @Deprecated
  public static GenericGamepad from(int port, boolean unused) {
    return new GenericGamepad(port);
  }

  public boolean periodic() {
    var name = DriverStation.getJoystickName(port);
    var unconnected = !DriverStation.isJoystickConnected(port);
    var xbox = DriverStation.getJoystickIsXbox(port);

    currentType =
        unconnected
            ? ControllerType.Unconnected
            : xbox
                ? ControllerType.Xbox
                : name.startsWith("Logitech Dual")
                    ? ControllerType.Logi
                    : name.startsWith("Wireless") ? ControllerType.PS4 : defaultType;

    return true;
  }

  // Magic hacks to make it run automatically
  // todo: see if this actually works or not?
  private final Trigger periodicTrigger = new Trigger(this::periodic);

  public int typeSwitch(int ps4, int xbox, int logi) {
    return switch (currentType) {
      case Unconnected -> -1;
      case PS4 -> ps4;
      case Xbox -> xbox;
      case Logi -> logi;
    };
  }

  public Trigger button(PS4Controller.Button ps4, XboxController.Button xbox, Integer logi) {
    return new Trigger(b(ps4, xbox, logi));
  }

  public BooleanSupplier b(PS4Controller.Button ps4, XboxController.Button xbox, Integer logi) {
    return () -> {
      var buttonValue = typeSwitch(ps4.value, xbox.value, logi);
      // if button value < 1, it is invalid
      return buttonValue > 0 && DriverStation.getStickButton(port, buttonValue);
    };
  }

  public DoubleSupplier axis(PS4Controller.Axis ps4, XboxController.Axis xbox, Integer logi) {
    return () -> {
      var axisValue = typeSwitch(ps4.value, xbox.value, logi);
      // if axis value < 0, it is invalid
      return axisValue >= 0 ? DriverStation.getStickAxis(port, axisValue) : 0;
    };
  }
}