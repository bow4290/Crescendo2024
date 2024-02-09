package frc.robot;

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
        Trigger intakeSpinController = controller.leftTriggerB;
        double intakeSpeedController = controller.leftTrigger.getAsDouble();
        boolean intakeDirectionController =  controller.leftBumper.getAsBoolean();

        //Shooter Controls
        Trigger shooterSpinController = controller.leftTriggerB;
        double shooterSpeedController = controller.leftTrigger.getAsDouble();
        boolean shooterDirectionController =  controller.leftBumper.getAsBoolean();

        intakeSpinController.whileTrue(bot.intake.spinMotor(intakeDirectionController ? -1 : 1, intakeSpeedController));
        shooterSpinController.whileTrue(bot.shooter.spinMotor(shooterDirectionController ? -1 : 1, shooterSpeedController));
        
    }
    
}
