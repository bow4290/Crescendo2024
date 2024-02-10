package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;

public class Shooter extends SubsystemBase{
  public static final int SHOOTER_MOTOR_ID = 13;

  private TalonFX shooterMotor;
  private int direction = 1;
  private double speed = 0;

  
  public Shooter() {
    shooterMotor = new TalonFX(SHOOTER_MOTOR_ID);
  }

  private void stopMotor() {
    this.shooterMotor.stopMotor();  
  }

  public Command spinMotor() {
    return this.runEnd(() -> this.shooterMotor.set(this.direction), () -> this.stopMotor());
  }

  public Command setMotorDirection(){
    return this.startEnd(() -> this.direction = -1, () -> this.direction = 1);
  }

  public Command setMotorSpeed(double speed){
    return this.runEnd(() -> this.speed = speed, () -> this.speed = 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Set Shooter Speed", this.speed);
    SmartDashboard.putNumber("Shooter Speed", this.shooterMotor.get());
    SmartDashboard.putNumber("Shooter Direction", this.direction);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
