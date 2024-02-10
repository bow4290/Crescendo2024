package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;

public class Shooter extends SubsystemBase{
  public static final int SHOOTER_MOTOR_ID = 10;

  private TalonFX shooterMotor;

  
  public Shooter() {
    shooterMotor = new TalonFX(SHOOTER_MOTOR_ID);
  }

  private void stopMotor() {
    this.shooterMotor.stopMotor();  
  }

  public Command spinMotor(int direction, double speed) {
    return this.runEnd(() -> this.shooterMotor.set(speed * direction / ((direction < 0) ? 10 : 1)), () -> this.stopMotor());
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
