package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {
  
  //Subsystem Constants
  public static final int INTAKE_MOTOR_ID = 9;

  private TalonFX intakeMotor;

  public Intake() {
    intakeMotor = new TalonFX(INTAKE_MOTOR_ID);
  }

  private void stopMotor() {
    this.intakeMotor.stopMotor();
  }

  public Command spinMotor(int direction, double speed) {
    return this.runEnd(() -> {this.intakeMotor.set(speed * direction);}, () -> {this.stopMotor();});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Speed", this.intakeMotor.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
