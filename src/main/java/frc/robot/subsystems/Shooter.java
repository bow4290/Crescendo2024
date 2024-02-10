package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;

public class Shooter extends SubsystemBase{
  public static final int SHOOTER_MOTOR_ID = 13;

  private TalonFX shooterMotor;

  
  public Shooter() {
    shooterMotor = new TalonFX(SHOOTER_MOTOR_ID);
  }

  private void stopMotor() {
    this.shooterMotor.stopMotor();  
  }

  public Command spinMotor(boolean direction, double speed) {
    if (direction) {
      return this.runEnd(() -> this.shooterMotor.set(speed * -1), () -> this.stopMotor());
    } else {
      return this.runEnd(() -> this.shooterMotor.set(speed), () -> this.stopMotor());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", this.shooterMotor.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
