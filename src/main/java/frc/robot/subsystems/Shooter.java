package frc.robot.subsystems;


//he he he

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.WrivotStates.BotAngleState;

public class Shooter extends SubsystemBase {
  // Subsystem Constants
  public static final int MOTOR_ID_SHOOTER = 10;
  public static final int MOTOR_ID_INDEXER = 16;

  public static final double SHOOTER_IN_SPEED = 0.4; // TODO: adjust Shooter in Speed to be more accurate
  public static final double INDEXER_IN_SPEED = 0.3; // TODO: adjust Indexer in Speed to be more accurate

  public static final double SHOOTER_OUT_SPEED = 0.5;
  public static final double INDEXER_OUT_SPEED = 0.25; // TODO: adjust Indexer out Speed to be more accurate

  private TalonFX motorShooter = new TalonFX(MOTOR_ID_SHOOTER);
  private TalonFX motorIndexer = new TalonFX(MOTOR_ID_INDEXER);

  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  //A command to move note from intake to indexer
  public Command cmdIndexIn(WrivotStates requireWrivotStates){
      StartEndCommand cmd = new StartEndCommand(
      () -> {
        motorShooter.setControl(dutyCycleOut.withOutput(SHOOTER_IN_SPEED));
        motorIndexer.setControl(dutyCycleOut.withOutput(INDEXER_IN_SPEED));
      }, 
      () -> {
        motorShooter.stopMotor();
        motorIndexer.stopMotor();
      }, this, requireWrivotStates);

      return cmd;
  }
  
  //A command to move note from indexer to be shot
  public Command cmdShootOut(WrivotStates requireWrivotStates) {

    // Guard clause to stop shooting out at intake position
    if (requireWrivotStates.getCurrentState() == BotAngleState.INTAKE){
      return this.startEnd(() -> {}, () -> {});
    }

    StartEndCommand cmd = new StartEndCommand(
      () -> {
        //Set speed of fly wheeeeeeels (yippee), then wait until they are sped up, 
        //(1 second at the time of this comment), then set index to move note to be shot. 
        motorShooter.setControl(dutyCycleOut.withOutput(SHOOTER_OUT_SPEED));
        Commands.waitSeconds(1);
        motorIndexer.setControl(dutyCycleOut.withOutput(INDEXER_OUT_SPEED));
      },
      () -> {
        motorShooter.stopMotor();
        motorIndexer.stopMotor();
      }, this, requireWrivotStates);

      return cmd;
  }


  @Override
  public void periodic(){
    SmartDashboard.putNumber("Current Shooter Speed", motorShooter.get());
    SmartDashboard.putNumber("Current Shooter Velocity", motorShooter.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("Current Indexer Speed", motorIndexer.get());
    SmartDashboard.putNumber("Current Index Velocity", motorIndexer.getVelocity().getValueAsDouble());

  }
    
}
































//I see you've found me