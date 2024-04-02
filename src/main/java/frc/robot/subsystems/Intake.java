package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase {

  public static final int MOTOR_ID_INTAKE = 9;
  public static final int NOTE_SENSOR_DIO_ID = 0;

  public static final double INTAKE_IN_SPEED = 0.5;
  public static final double INTAKE_INDEX_SPEED = 0.4;
  public static final double INTAKE_INDEX_BACK_SPEED = -0.15;
  public static final double INTAKE_DROP_SPEED = -0.6;

  private TalonFX motorIntake = new TalonFX(MOTOR_ID_INTAKE);
  private DigitalInput noteSensor = new DigitalInput(NOTE_SENSOR_DIO_ID);
  public Trigger noteTrigger = new Trigger(noteSensor::get);

  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public Command cmdSmartIntake(){
    return cmdIntakeIn().until(() -> isNoteIndexed());
  }
  
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

  public void runIntakeBackup(){
    motorIntake.set(INTAKE_INDEX_BACK_SPEED);
  }

  public void intakeStop(){
    motorIntake.stopMotor();
  }

  public boolean isNoteIndexed(){
    return !noteSensor.get();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Intake Output", motorIntake.get());
  }

}
