package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class CmdTorqueTester extends Command {
  private final Shooter subsystemShooter;
  private final Intake subsystemIntake;

  private int cycleTally;

  public CmdTorqueTester(Shooter subsystemShooter, Intake subsystemIntake){
    this.subsystemShooter = subsystemShooter;
    this.subsystemIntake = subsystemIntake;
  }

  @Override
  public void initialize() {
    cycleTally = 0;
  }

  @Override
  public void execute() {
    cycleTally++;
    if (cycleTally < 10){
      subsystemShooter.runTorqueTest();
    } else {
      if (subsystemShooter.isTorqueNote()){
        subsystemIntake.runIntakeBackup();
      } else {
        // This should instantly end the command.
        cycleTally = 18;
      }
    }


  }

  @Override
  public boolean isFinished(){
    return (cycleTally > 14);
  }

  @Override
  public void end(boolean interrupted){
    subsystemIntake.intakeStop();
    subsystemShooter.shooterStop();
  }

  
  
}
