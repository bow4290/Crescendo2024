package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  // Subsystem Constants
  public static final int MOTOR_ID_INTAKE = 9;

  public static final double INTAKE_IN_SPEED = 0.45; // TODO: adjust Intake in Speed to be more accurate
  public static final double INTAKE_OUT_SPEED = -0.90; // TODO: adjust Intake out Speed to be more accurate

  private TalonFX motorIntake = new TalonFX(MOTOR_ID_INTAKE);
  
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public Command cmdIntakeIn(){

    StartEndCommand cmd = new StartEndCommand(
      () -> {
        motorIntake.setControl(dutyCycleOut.withOutput(INTAKE_IN_SPEED));
      },
      () -> {
        motorIntake.stopMotor();
      }, this);

    return cmd;
  }

  public Command cmdIntakeOut(){
    StartEndCommand cmd = new StartEndCommand(() -> {
      motorIntake.setControl(dutyCycleOut.withOutput(INTAKE_OUT_SPEED));
    },
    () -> {
      motorIntake.stopMotor();
    }, this);

    return cmd;

  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Intake Output", motorIntake.get());
    SmartDashboard.putNumber("Current Intake Velocity", motorIntake.getVelocity().getValueAsDouble());
  }
    
}
