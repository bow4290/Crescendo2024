package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.lib.GenericGamepad;
import frc.lib.math.Conversions;
import frc.robot.StateManager.BotState;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;


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
        () -> -controller.rightX.getAsDouble(),
        () -> controller.rightBumper.getAsBoolean()
    ));

    // Dpad Up - Climber Up (Ready)
    controller.dpadUp.whileTrue(bot.climber.cmdClimbTogether(Climber.CLIMBER_UP_SPEED));

    // Dpad Down - Climber Down (Chin-up)
    controller.dpadDown.whileTrue(bot.climber.cmdClimbTogether(Climber.CLIMBER_DOWN_SPEED));

  }

  // Operator Controls:
  // Left Bumper - Intake
  // Right Bumper - Drop (Intake out)
  // Left Trigger - Shoot
  // Right Trigger - Aim (NOT YET IMPLEMENTED)
  // Cross / A - Cancel Wrivot
  // Dpad Right - State Stash
  // Dpad Down - State Intake
  // Dpad Left - State Amp
  // Dpad Up - State Speaker Base
  public static void configureOperator(RobotContainer bot){
    GenericGamepad controller = bot.controllerOperator;

    // Smart intake
    controller.leftBumper.whileTrue(bot.intake.cmdSmartIntake());

    // Drop (intake out)
    controller.rightBumper.whileTrue(bot.intake.cmdIntakeDrop());

    // Shoot Out
    controller.rightTriggerB.whileTrue(autoShootOut(bot));

    // Cancel Pivot + Wrist actions and PID
    controller.cross_a.onTrue(Commands.runOnce(() -> bot.pivot.pivotStop()).andThen(() -> bot.wrist.wristStop()));

    // Set State: Stash
    controller.dpadRight.onTrue(fullSequence(BotState.STASH, bot).until(() -> controller.cross_a.getAsBoolean()));

    // Set State: Intake
    controller.dpadDown.onTrue(fullSequence(BotState.INTAKE, bot).until(() -> controller.cross_a.getAsBoolean()));

    // Set State: Speaker Base
    controller.dpadUp.onTrue(fullSequence(BotState.SPEAKER_BASE, bot).until(() -> controller.cross_a.getAsBoolean()));

    // Set State: Amp
    controller.dpadLeft.onTrue(fullSequence(BotState.AMP, bot).until(() -> controller.cross_a.getAsBoolean()));
  }

  public static Command autoShootOut(RobotContainer botInstance){
    return Commands.parallel(
      botInstance.shooter.cmdShootOut(),
      Commands.waitUntil(() -> botInstance.shooter.isShooterSpeed(Shooter.SHOOTER_TOP_OUT_RPM)).withTimeout(0.5).andThen(botInstance.intake.cmdIntakeIn())
    );
  }

  public static Command fullSequence(BotState targetState, RobotContainer botInstance){
    boolean condition = botInstance.pivot.getPivotMotorRotations() > Conversions.degToRotations(8);
    return Commands.sequence(
      botInstance.wrist.cmdWristToDeg(BotState.STASH.wristDegrees).onlyIf(() -> condition).until(() -> botInstance.wrist.isWristFinished(BotState.STASH.wristDegrees)),
      botInstance.pivot.cmdPivotToDeg(targetState.pivotDegrees).until(() -> botInstance.pivot.isPivotFinished(targetState.pivotDegrees)),
      botInstance.wrist.cmdWristToDeg(targetState.wristDegrees).until(() -> botInstance.wrist.isWristFinished(targetState.wristDegrees))
      );
  }

  public static Command wristCentricSequence(BotState targetState, RobotContainer botInstance){
    return Commands.sequence(
      botInstance.pivot.cmdPivotToDeg(targetState.pivotDegrees).until(() -> botInstance.pivot.isPivotFinished(targetState.pivotDegrees)),
      botInstance.wrist.cmdWristToDeg(targetState.wristDegrees).until(() -> botInstance.wrist.isWristFinished(targetState.wristDegrees))
    );
  }
    
}
