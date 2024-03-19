package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.lib.GenericGamepad;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.NewWrivot.BotStates;
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

    controller.rightTriggerB.onTrue(bot.led.setLedsToSolidColor(Color.kWhite));
    controller.rightTriggerB.onFalse(bot.led.setLedsToSolidColor(Color.kBlack));

  }

  // Operator Controls:
  
  // Dpad Right - State Stash
  // Dpad Down - State Intake
  // Dpad Left - State Amp
  // Dpad Up - State Shoot
  // Cross / A - Cancel Wrivot Action
  // Left Trigger - Intake (All)
  // Right Trigger - Shoot
  public static void configureOperator(RobotContainer bot){
    GenericGamepad controller = bot.controllerOperator;

    // Cancel Wrivot Action
    controller.cross_a.onTrue(Commands.runOnce(() -> bot.wrivot.endMotorRequests()));

    // Dpad Right - State Stash
    controller.dpadRight.onTrue(bot.wrivot.cmdGoToState(BotStates.STASH)
    .until(() -> controller.cross_a.getAsBoolean())
    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Dpad Down - State Intake
    controller.dpadDown.onTrue(bot.wrivot.cmdGoToState(BotStates.INTAKE)
    .until(() -> controller.cross_a.getAsBoolean())
    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Dpad Left - State Amp
    controller.dpadLeft.onTrue(bot.wrivot.cmdGoToState(BotStates.AMP)
    .until(() -> controller.cross_a.getAsBoolean())
    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Dpad Up - State Speaker
    controller.dpadUp.onTrue(bot.wrivot.cmdGoToState(BotStates.SPEAKER)
    .until(() -> controller.cross_a.getAsBoolean())
    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Options - Manual Pivot Up 
    controller.rightMiddle.whileTrue(bot.wrivot.cmdDrivePivot(0.4));
    // Share - Manual Pivot Down
    controller.leftMiddle.whileTrue(bot.wrivot.cmdDrivePivot(-0.3));

    // L3 / Left Joystick Push - Zero Wrivot
    controller.leftJoystickPushed.onTrue(new InstantCommand(() -> bot.wrivot.zeroWrivot()));

    // Left Bumper - Intake Grab
    controller.leftBumper.whileTrue(bot.intake.cmdIntakeGrab());

    // Right Bumper - Intake Drop / Throw
    controller.rightBumper.whileTrue(bot.intake.cmdIntakeOut());

    // Left Trigger - Intake
    controller.leftTriggerB.onTrue(Commands.parallel(
      bot.intake.cmdIntakeIn().until(() -> {return !controller.leftTriggerB.getAsBoolean();}),
      bot.shooter.cmdShooterIntake().until(() -> {return !controller.leftTriggerB.getAsBoolean();})
      ));
    
    // Circle / B - Shooter Prespin
    controller.circle_b.onTrue(bot.shooter.cmdStartShooter(Shooter.SHOOTER_PRESPIN_RPM));
      
    // Right Trigger - Shoot
    controller.rightTriggerB.whileTrue(bot.shooter.cmdShootOut());
    

  }
    
}
