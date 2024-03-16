package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NewWrivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.NewWrivot.BotStates;

public class AutoCommands {
  private Shooter auto_shooter;
  private Intake auto_intake;
  private Swerve auto_swerve;
  private NewWrivot auto_wrivot;

  public AutoCommands(Shooter auto_shooter, Intake auto_intake, Swerve auto_swerve, NewWrivot auto_wrivot) {
    this.auto_shooter = auto_shooter;
    this.auto_intake = auto_intake;
    this.auto_swerve = auto_swerve;
    this.auto_wrivot = auto_wrivot;
  }

  public Command ShootSpeakerBase() {
    return new SequentialCommandGroup(auto_wrivot.cmdGoToState(BotStates.SPEAKER).withTimeout(3), Commands.waitSeconds(1), auto_shooter.cmdShootOut().withTimeout(3));
  }
  
}
