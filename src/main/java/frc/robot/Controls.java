package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.commands.*;

public class Controls {

    public void driverConfiguration(RobotContainer bot) {
        bot.s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                bot.s_Swerve, 
                () -> -bot.driver.leftY.getAsDouble(), 
                () -> -bot.driver.leftX.getAsDouble(), 
                () -> -bot.driver.rightX.getAsDouble(), 
                () -> bot.driver.leftBumper.getAsBoolean()
            )
        );
        bot.driver.y_triangle.onTrue(new InstantCommand(bot.s_Swerve::zeroHeading));
    }
}
