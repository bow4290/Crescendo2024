package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;

public class Shooter extends SubsystemBase {
  public static final int MOTOR_ID_SHOOTER = 10;

  public static final double SHOOTER_IN_RPM = -4000;
  public static final double SHOOTER_OUT_RPM = 6000;
  public static final double SHOOTER_PRESPIN_RPM = 2000;

  // TODO: Find if this is different?
  public static final double GEAR_RATIO_SHOOTER = 1/1.5;

  private TalonFX motorShooter = new TalonFX(MOTOR_ID_SHOOTER);

  private VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  public Shooter(){
    TalonFXConfiguration configurationShooter = new TalonFXConfiguration();
    configurationShooter.Slot0.kS = 0.05;
    configurationShooter.Slot0.kV = 0.12;
    configurationShooter.Slot0.kP = 2.1;
    configurationShooter.Slot0.kI = 0.0;
    configurationShooter.Slot0.kD = 0;
  }

  public Command cmdShootOut(){
    double targetRPS = Conversions.rpmToRpsGearRatio(SHOOTER_OUT_RPM, GEAR_RATIO_SHOOTER);

    return this.startEnd(() -> {
      motorShooter.setControl(velocityVoltage.withVelocity(targetRPS));
    }, () -> {
      motorShooter.stopMotor();
    });
  }
  
}
