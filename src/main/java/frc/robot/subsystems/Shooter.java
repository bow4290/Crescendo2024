package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.GenericGamepad;

public class Shooter extends SubsystemBase{
  public static final int SHOOTER_MOTOR_ID = 13;

  private TalonFX shooterMotor;

  private double speed = 0;
  
  public Shooter() {
    shooterMotor = new TalonFX(SHOOTER_MOTOR_ID);
  }

  private void stopMotor() {
    SmartDashboard.putNumber("POTATO", 0);
    this.shooterMotor.stopMotor();  
  }

  public Command spinMotor(DoubleSupplier speed) {
    SmartDashboard.putNumber("POTATO", speed.getAsDouble());
    return this.runEnd(() -> this.shooterMotor.set(speed.getAsDouble()), () -> this.stopMotor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", this.shooterMotor.get());
    SmartDashboard.putNumber("Shooter Speed MAYBE", this.shooterMotor.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
