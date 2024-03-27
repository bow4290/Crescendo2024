package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  public static final int MOTOR_ID_INTAKE = 9;

  public static final double INTAKE_IN_SPEED = 0.2;
  public static final double INTAKE_INDEX_SPEED = 0.4;
  public static final double INTAKE_DROP_SPEED = -0.6;

  private TalonFX motorIntake = new TalonFX(MOTOR_ID_INTAKE);

  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  
  /**
   * Runs intake in to intake.
   * @return Command
   */
  public Command cmdIntakeIn(){
    return this.startEnd(() -> {
      motorIntake.setControl(dutyCycleOut.withOutput(INTAKE_IN_SPEED));
    }, () -> {
      motorIntake.stopMotor();
    });
  }

  /**
   * Runs intake out to drop note.
   * @return Command
   */
  public Command cmdIntakeDrop(){
    return this.startEnd(() -> {
      motorIntake.setControl(dutyCycleOut.withOutput(INTAKE_DROP_SPEED));
    }, () -> {
      motorIntake.stopMotor();
    });
  }

  /**
   * Runs intake in to kick into shooter. 
   * @return Command
   */
  public Command cmdIndex(){
    return this.startEnd(() -> {
      motorIntake.setControl(dutyCycleOut.withOutput(INTAKE_INDEX_SPEED));
    }, () -> {
      motorIntake.stopMotor();
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Intake Output", motorIntake.get());
  }

}
