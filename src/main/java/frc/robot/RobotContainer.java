package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.GenericGamepad;
import frc.robot.autos.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public final GenericGamepad controllerDriver = GenericGamepad.from(0);
    public final GenericGamepad controllerOperator = GenericGamepad.from(1);


    /* Subsystems */
    public final Swerve swerve = new Swerve();
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final WrivotStates wrivotStates = new WrivotStates();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Configure the button bindings
        Controls.configureDriver(this);
        Controls.configureOperator(this);
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(swerve);
    }
}
