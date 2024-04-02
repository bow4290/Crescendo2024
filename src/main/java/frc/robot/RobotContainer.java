package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.*;

import frc.lib.GenericGamepad;
import frc.robot.StateManager.BotState;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final static double AUTO_TIMEOUT_INTAKE = 1.75;
    public final static double AUTO_TIMEOUT_DROP = 0.25;
    public final static double AUTO_SHOOT_TIMEOUT = 2.5;
    public final static double AUTO_STATE_TIMEOUT = 4.5;

    /* Controllers */
    public final GenericGamepad controllerDriver = GenericGamepad.from(0);
    public final GenericGamepad controllerOperator = GenericGamepad.from(1);


    /* Subsystems */
    public final Swerve swerve = new Swerve();
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final Climber climber = new Climber();
    public final Pivot pivot = new Pivot();
    public final Wrist wrist = new Wrist();
    public final LED led = new LED();
    
    private SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        Controls.configureDriver(this);
        Controls.configureOperator(this);
        Controls.configureRobot(this);

        NamedCommands.registerCommand("Auto Intake", intake.cmdSmartIntake());
        NamedCommands.registerCommand("Intake Drop", intake.cmdIntakeDrop().withTimeout(AUTO_TIMEOUT_DROP));
        NamedCommands.registerCommand("Shoot", Controls.autoShootOut(this).withTimeout(AUTO_SHOOT_TIMEOUT));

        NamedCommands.registerCommand("State Stash", Controls.fullSequence(BotState.STASH, this));
        NamedCommands.registerCommand("State Intake", Controls.fullSequence(BotState.INTAKE, this));
        NamedCommands.registerCommand("State Speaker", Controls.fullSequence(BotState.SPEAKER_BASE, this));
        NamedCommands.registerCommand("State Amp", Controls.fullSequence(BotState.AMP, this));

        autoChooser = AutoBuilder.buildAutoChooser();

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

    public void robotContainerPeriodic(){
      SmartDashboard.putNumber("Limelight Distance to Speaker", Vision.getLimelightDistanceToTag(Vision.SPEAKER_APRIL_TAG_HEIGHT));
    }
}
