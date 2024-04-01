package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.lib.GenericGamepad;

import frc.robot.StateManager.BotState;
import frc.robot.commands.CmdTorqueTester;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.commands.CmdTorqueTester;

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

    // Swerve w/ Right Bumper as RobotCentric
    bot.swerve.setDefaultCommand(new TeleopSwerve(            
        bot.swerve, 
        () -> -controller.leftY.getAsDouble(),
        () -> -controller.leftX.getAsDouble(),
        () -> getWantedRotationalValue(false, -controller.rightX.getAsDouble(), 0.0, controller),
        () -> controller.rightBumper.getAsBoolean()
    ));

    // Dpad Up - Climber Up (Ready)
    controller.dpadUp.whileTrue(bot.climber.cmdClimbTogether(Climber.CLIMBER_UP_SPEED));

    // Dpad Down - Climber Down (Chin-up)
    controller.dpadDown.whileTrue(bot.climber.cmdClimbTogether(Climber.CLIMBER_DOWN_SPEED));

  }

  public static void configureOperator(RobotContainer bot){
    GenericGamepad controller = bot.controllerOperator;

    // Smart intake
    controller.leftBumper.whileTrue(bot.intake.cmdSmartIntake());

    // Drop (intake out)
    controller.rightBumper.whileTrue(bot.intake.cmdIntakeDrop());

    // Shoot Out
    controller.rightTriggerB.whileTrue(Commands.parallel(
      new CmdTorqueTester(bot.shooter, bot.intake).andThen(
        bot.shooter.cmdShootOut()
      ),
      Commands.waitUntil(() -> bot.shooter.isShooterSpeed(bot.shooter.SHOOTER_OUT_RPM)).andThen(bot.intake.cmdIntakeIn())
    ));

    // Cancel Pivot + Wrist actions and PID
    controller.cross_a.onTrue(Commands.runOnce(() -> bot.pivot.pivotStop()).andThen(() -> bot.wrist.wristStop()));

    // Set State: Stash
    controller.dpadRight.onTrue(wrivotSequence(BotState.STASH, bot));

    // Set State: Intake
    controller.dpadDown.onTrue(wrivotSequence(BotState.INTAKE, bot));

    // Set State: Speaker Base
    controller.dpadUp.onTrue(wrivotSequence(BotState.SPEAKER_BASE, bot));

    // Set State: Amp
    controller.dpadLeft.onTrue(wrivotSequence(BotState.AMP, bot));
  }

  public static Command wrivotSequence(BotState targetState, RobotContainer botInstance){
    return Commands.sequence(
      // botInstance.wrist.cmdWristToDeg(BotState.STASH.wristDegrees).until(() -> botInstance.wrist.isWristFinished(BotState.STASH.wristDegrees)),
      // botInstance.pivot.cmdPivotToDeg(targetState.pivotDegrees).until(() -> botInstance.pivot.isPivotFinished(targetState.pivotDegrees)),
      botInstance.wrist.cmdWristToDeg(targetState.wristDegrees).until(() -> botInstance.wrist.isWristFinished(targetState.wristDegrees))
      );
  }

  private static double getWantedRotationalValue(boolean disregardJoystick, double joystickValue, double wantedRotation, GenericGamepad controller){
    return (disregardJoystick) ? 
        0
      : 
        joystickValue;
  }
    
}
