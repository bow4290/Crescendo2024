package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  // Subsystem Constants
  public static final int MOTOR_ID_INTAKE = 9;

  public static final double INTAKE_IN_SPEED = 0.45; // TODO: adjust Intake in Speed to be more accurate
  public static final double INTAKE_OUT_SPEED = 0.40; // TODO: adjust Intake out Speed to be more accurate


  private TalonFX motorIntake = new TalonFX(MOTOR_ID_INTAKE);

  public Command cmdIntakeIn(){
      return this.runEnd(
      () -> {
        motorIntake.set(INTAKE_IN_SPEED);
      }, 
      () -> {
        motorIntake.stopMotor();
      }
    );
  }

  public Command cmdIntakeOut(){
    return this.runEnd(
      () -> {
        motorIntake.set(INTAKE_OUT_SPEED);
      },
      () -> {
        motorIntake.stopMotor();
      });
  }
    
}
