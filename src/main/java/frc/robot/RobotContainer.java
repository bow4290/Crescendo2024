package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.commands.*;

import frc.lib.GenericGamepad;
import frc.robot.subsystems.*;
import frc.robot.subsystems.NewWrivot.BotStates;
import frc.robot.autos.AutoCommands;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final static double AUTO_TIMEOUT_INTAKE = 1.75;
    public final static double AUTO_TIMEOUT_GRAB = 0.15;
    public final static double AUTO_TIMEOUT_DROP = 1;
    public final static double AUTO_SHOOT_TIMEOUT = 2.25;
    public final static double AUTO_STATE_TIMEOUT = 4.5;

    /* Controllers */
    public final GenericGamepad controllerDriver = GenericGamepad.from(0);
    public final GenericGamepad controllerOperator = GenericGamepad.from(1);


    /* Subsystems */
    public final Swerve swerve = new Swerve();
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final NewWrivot wrivot = new NewWrivot();
    public final Climber climber = new Climber();
    public final LED led = new LED();
    
    private SendableChooser<Command> autoChooser;
    private AutoCommands autoCommands;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        autoCommands = new AutoCommands(shooter, intake, swerve, wrivot);
        // Configure the button bindings
        Controls.configureDriver(this);
        Controls.configureOperator(this);

        // Register Name Commands
        NamedCommands.registerCommand("Intake Full", Commands.parallel(
          intake.cmdIntakeIn(),
          shooter.cmdShooterIntake()
        ).withTimeout(AUTO_TIMEOUT_INTAKE));

        NamedCommands.registerCommand("Intake Grab", intake.cmdIntakeGrab().withTimeout(AUTO_TIMEOUT_GRAB));
        NamedCommands.registerCommand("Intake Drop", intake.cmdIntakeOut().withTimeout(AUTO_TIMEOUT_DROP));

        NamedCommands.registerCommand("Prespin Shooter", shooter.cmdStartShooter(Shooter.SHOOTER_PRESPIN_RPM));
        NamedCommands.registerCommand("Stop Shooter", shooter.cmdStopShooter());

        NamedCommands.registerCommand("Intake", Commands.parallel(
          intake.cmdIntakeIn(),
          shooter.cmdShooterIntake()
        ).withTimeout(AUTO_TIMEOUT_INTAKE));
        NamedCommands.registerCommand("Shoot", shooter.cmdShootOut().withTimeout(AUTO_SHOOT_TIMEOUT));

        NamedCommands.registerCommand("Go To Stash", wrivot.cmdGoToState(BotStates.STASH).withTimeout(AUTO_STATE_TIMEOUT));
        NamedCommands.registerCommand("Go To Intake", wrivot.cmdGoToState(BotStates.INTAKE).withTimeout(AUTO_STATE_TIMEOUT));
        NamedCommands.registerCommand("Go To Speaker", wrivot.cmdGoToState(BotStates.SPEAKER).withTimeout(AUTO_STATE_TIMEOUT));
        NamedCommands.registerCommand("Go To Amp", wrivot.cmdGoToState(BotStates.AMP).withTimeout(AUTO_STATE_TIMEOUT));

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("HARDCODED Speaker Base Shoot", autoCommands.ShootSpeakerBase());

        SmartDashboard.putData("Auto Picker", autoChooser);
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      // An ExampleCommand will run in autonomous
      return autoChooser.getSelected();
    }
}
