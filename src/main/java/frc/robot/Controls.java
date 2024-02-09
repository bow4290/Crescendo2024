package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.GenericGamepad;
import frc.robot.commands.TeleopSwerve;

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
        //Intake Controls
        final Trigger intakeSpinController = controller.leftTriggerB;
        final DoubleSupplier intakeSpeedController = controller.leftTrigger;
        final BooleanSupplier intakeDirectionController =  controller.leftBumper;

        //Shooter Controls
        final Trigger shooterSpinController = controller.leftTriggerB;
        final DoubleSupplier shooterSpeedController = controller.leftTrigger;
        final BooleanSupplier shooterDirectionController =  controller.leftBumper;

        intakeSpinController.whileTrue(bot.intake.spinMotor(intakeDirectionController.getAsBoolean() ? -1 : 1, intakeSpeedController.getAsDouble()));
        shooterSpinController.whileTrue(bot.shooter.spinMotor(shooterDirectionController.getAsBoolean() ? -1 : 1, shooterSpeedController.getAsDouble()));
        
    }
    
}
