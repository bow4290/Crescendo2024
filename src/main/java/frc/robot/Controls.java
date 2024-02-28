package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.lib.GenericGamepad;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.WrivotStates.BotAngleState;

public class Controls {

  // Driver Controls:
  // Left Stick - Drive
  // Right Stick - Rotate / Swerve
  // Right Bumper - Bot Relative
  // Triangle / Y - Zero Gyro
  public static void configureDriver(RobotContainer bot){
    GenericGamepad controller = bot.controllerDriver;

    // Zero Gyro
    controller.triangle_y.onTrue(new InstantCommand(bot.swerve::zeroHeading));

    // Swerve w/ Right Trigger as RobotCentric
    bot.swerve.setDefaultCommand(new TeleopSwerve(            
        bot.swerve, 
        () -> controller.leftY.getAsDouble(),
        () -> controller.leftX.getAsDouble(),
        () -> controller.rightX.getAsDouble(),
        () -> controller.rightBumper.getAsBoolean()
    ));

  }

  // Operator Controls:
  // Dpad Right - State Stash
  // Dpad Down - State Intake
  // Dpad Left - State Amp
  // Dpad Up - State Shoot
  // Cross / A - Cancel Wrivot Action
  public static void configureOperator(RobotContainer bot){
    GenericGamepad controller = bot.controllerOperator;

    // Cancel Wrivot Action
    controller.cross_a.onTrue(Commands.runOnce(() -> bot.wrivotStates.endMotorRequests()));

    // Go to state: stash
    controller.dpadRight.onTrue(bot.wrivotStates.cmdWrivotSequencer(BotAngleState.STASH)
    .until(() -> controller.cross_a.getAsBoolean())
    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Go to state: intake
    controller.dpadDown.onTrue(bot.wrivotStates.cmdWrivotSequencer(BotAngleState.INTAKE)
    .until(() -> controller.cross_a.getAsBoolean())
    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Go to state: amp
    controller.dpadLeft.onTrue(bot.wrivotStates.cmdWrivotSequencer(BotAngleState.AMP)
    .until(() -> controller.cross_a.getAsBoolean())
    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Go to state: shooter
    controller.dpadUp.onTrue(bot.wrivotStates.cmdWrivotSequencer(BotAngleState.SPEAKER)
    .until(() -> controller.cross_a.getAsBoolean())
    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
  }
    
}
