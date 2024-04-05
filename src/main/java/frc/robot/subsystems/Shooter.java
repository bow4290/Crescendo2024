package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;

public class Shooter extends SubsystemBase {
  public static final int MOTOR_ID_SHOOTER_BOTTOM = 10;
  public static final int MOTOR_ID_SHOOTER_TOP = 18;

  public static final double SHOOTER_IN_RPM = -4000;
  public static final double SHOOTER_BOTTOM_OUT_RPM = 6000;
  public static final double SHOOTER_TOP_OUT_RPM = 6000;
  public static final double SHOOTER_PRESPIN_RPM = 6000;
  public static final double SHOOTER_TEST_OUT = 0.05;

  public static final double GEAR_RATIO_SHOOTER = 1/1.5;

  public static final double TOLERANCE = 10;

  private TalonFX motorShooterBottom = new TalonFX(MOTOR_ID_SHOOTER_BOTTOM);
  private TalonFX motorShooterTop = new TalonFX(MOTOR_ID_SHOOTER_TOP);

  private VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public Shooter(){
    TalonFXConfiguration configurationShooter = new TalonFXConfiguration();

    configurationShooter.Feedback.SensorToMechanismRatio = GEAR_RATIO_SHOOTER;

    configurationShooter.Slot0.kS = 0.1;
    configurationShooter.Slot0.kV = 0.12;
    configurationShooter.Slot0.kP = 3;
    configurationShooter.Slot0.kI = 0.0;
    configurationShooter.Slot0.kD = 0;

    motorShooterBottom.getConfigurator().apply(configurationShooter);
    motorShooterTop.getConfigurator().apply(configurationShooter);
  }


  public Command cmdShootOut(){
    double targetBottomRPS = Conversions.rpmToRps(SHOOTER_BOTTOM_OUT_RPM);
    double targetTopRPS = Conversions.rpmToRps(SHOOTER_TOP_OUT_RPM);

    double targetDutyCycle = SHOOTER_TOP_OUT_RPM / 9000;

    return this.startEnd(() -> {
      motorShooterBottom.setControl(velocityVoltage.withVelocity(targetBottomRPS));
      motorShooterTop.setControl(dutyCycleOut.withOutput(targetDutyCycle));
    }, () -> {
      shooterStop();
    });
  }

  public void shooterStop(){
    motorShooterBottom.stopMotor();
    motorShooterTop.stopMotor();
  }

  public boolean isShooterSpeed(double targetSpeed){
    return (MathUtil.isNear(targetSpeed, motorShooterTop.getVelocity().getValueAsDouble() * 60, TOLERANCE));

  }


  @Override
  public void periodic(){
    SmartDashboard.putNumber("Current Shooter 1 RPM", motorShooterBottom.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("Current Shooter 2 RPM", motorShooterTop.getVelocity().getValueAsDouble() * 60);

    SmartDashboard.putNumber("Current Shooter 1 Torque Current", motorShooterBottom.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Current Shooter 2 Torque Current", motorShooterTop.getTorqueCurrent().getValueAsDouble());
  }
  
}
