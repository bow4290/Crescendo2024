package frc.robot.subsystems;


//he he he

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

  public Command cmdShootOut(WrivotStates requireWrivotStates){
    return Commands.parallel(
      cmdRunShooter(SHOOTER_OUT_SPEED),
      cmdRunIndexer(INDEXER_OUT_SPEED)
    );
  }

  public Command cmdShooterIntake(){
    return Commands.parallel(
      cmdRunShooter(SHOOTER_IN_SPEED),
      cmdRunIndexer(INDEXER_IN_SPEED)
    );
  }

  private Command cmdRunIndexer(double targetSpeed){
    return this.runEnd(
    () -> {
      motorIndexer.setControl(dutyCycleOut.withOutput(targetSpeed));
    },
    () -> {
      motorIndexer.stopMotor();
    });
  }

  private Command cmdRunShooter(double targetSpeed){
    return this.startEnd(
    () -> {
      motorShooter.setControl(dutyCycleOut.withOutput(targetSpeed));
    },
    () -> {
      motorShooter.stopMotor();
    });
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