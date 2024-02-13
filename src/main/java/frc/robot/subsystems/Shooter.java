package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.swerve.SwerveModule;

public class Shooter extends SubsystemBase {
  // Subsystem Constants
  public static final int MOTOR_ID_SHOOTER = 10;
  public static final int MOTOR_ID_INDEXER = 16;

  public static final double SHOOTER_IN_SPEED = 0.45; // TODO: adjust Shooter in Speed to be more accurate
  public static final double INDEXER_IN_SPEED = 0.45; // TODO: adjust Indexer in Speed to be more accurate

  public static final double INDEXER_OUT_SPEED = 0.4; // TODO: adjust Indexer out Speed to be more accurate
  public static enum SHOOTER_OUT_SPEEDS { // TODO: adjust Shooter out Speed to be more accurate
    SHOOTER_OUT_SPEED_ONE,
    SHOOTER_OUT_SPEED_TWO
  }

  public double ShooterEnumToDoubleSpeed(SHOOTER_OUT_SPEEDS speed) {
    switch(speed){
      case SHOOTER_OUT_SPEED_ONE:
        return 1.0;
      case SHOOTER_OUT_SPEED_TWO:
        return 0.4;
      default:
        return 0.0;
    }
  }

  private TalonFX motorShooter = new TalonFX(MOTOR_ID_SHOOTER);
  private TalonFX motorIndexer = new TalonFX(MOTOR_ID_INDEXER);

  public Command cmdIndexIn(){
      return this.runEnd(
      () -> {
        motorShooter.set(SHOOTER_IN_SPEED);
        motorIndexer.set(INDEXER_IN_SPEED);
      }, 
      () -> {
        motorShooter.stopMotor();
        motorIndexer.stopMotor();
      }
    );
  }

  public Command cmdShootOut() {
    return this.runEnd(
      () -> {
        motorShooter.set(ShooterEnumToDoubleSpeed(SHOOTER_OUT_SPEEDS.SHOOTER_OUT_SPEED_TWO));
        Commands.waitSeconds(1);
        motorIndexer.set(INDEXER_OUT_SPEED);
      },
      () -> {
        motorShooter.stopMotor();
        motorIndexer.stopMotor();
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
