package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.GenericGamepad;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.WrivotStates.BotAngleState;

public class Controls {

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
            () -> controller.rightTriggerB.getAsBoolean()
        ));
 
    }

    public static void configureOperator(RobotContainer bot){
        GenericGamepad controller = bot.controllerOperator;

        controller.dpadRight.onTrue(bot.wrivotStates.cmdGoToState(BotAngleState.STASH));
        controller.dpadUp.onTrue(bot.wrivotStates.cmdGoToState(BotAngleState.SPEAKER));

    }
    
}
