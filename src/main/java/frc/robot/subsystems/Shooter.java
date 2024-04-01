package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;

public class Shooter extends SubsystemBase {
  public static final int MOTOR_ID_SHOOTER_1 = 10;
  public static final int MOTOR_ID_SHOOTER_2 = 18; // TODO: Find what motor id we are using

  public static final double SHOOTER_IN_RPM = -4000;
  public static final double SHOOTER_OUT_RPM = 6000;
  public static final double SHOOTER_PRESPIN_RPM = 2000;
  public static final double SHOOTER_TEST_OUT = 0.05;

  // TODO: Find if this is different?
  public static final double GEAR_RATIO_SHOOTER = 1/1.5;

  public static final double NOTE_TORQUE_THRESHOLD = 20;
  public static final double TOLERANCE = 10;

  private TalonFX motorShooter1 = new TalonFX(MOTOR_ID_SHOOTER_1);
  private TalonFX motorShooter2 = new TalonFX(MOTOR_ID_SHOOTER_2);

  private VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  private LinearFilter torqueFilter = LinearFilter.movingAverage(8);
  private double currentTorqueFiltered;

  public Shooter(){
    TalonFXConfiguration configurationShooter = new TalonFXConfiguration();

    configurationShooter.Feedback.SensorToMechanismRatio = GEAR_RATIO_SHOOTER;

    configurationShooter.Slot0.kS = 0.05;
    configurationShooter.Slot0.kV = 0.12;
    configurationShooter.Slot0.kP = 2.1;
    configurationShooter.Slot0.kI = 0.0;
    configurationShooter.Slot0.kD = 0;

    motorShooter1.getConfigurator().apply(configurationShooter);
    motorShooter2.getConfigurator().apply(configurationShooter);
  }


  public Command cmdShootOut(){
    double targetRPS = Conversions.rpmToRps(SHOOTER_OUT_RPM);

    return this.startEnd(() -> {
      motorShooter1.setControl(velocityVoltage.withVelocity(targetRPS));
      motorShooter2.setControl(velocityVoltage.withVelocity(targetRPS));
    }, () -> {
      shooterStop();
    });
  }

  public void runTorqueTest(){
    motorShooter1.set(SHOOTER_TEST_OUT);
  }

  public void shooterStop(){
    motorShooter1.stopMotor();
    motorShooter2.stopMotor();
  }

  public boolean isShooterSpeed(double targetSpeed){
    return (MathUtil.isNear(targetSpeed, motorShooter1.getVelocity().getValueAsDouble() * 60, TOLERANCE));

  }

  public boolean isTorqueNote(){
    return (currentTorqueFiltered > NOTE_TORQUE_THRESHOLD);
  }

  @Override
  public void periodic(){
    currentTorqueFiltered = torqueFilter.calculate(motorShooter1.getTorqueCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Current Shooter 1 RPM", motorShooter1.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("Current Shooter 2 RPM", motorShooter2.getVelocity().getValueAsDouble() * 60);

    SmartDashboard.putNumber("Current Shooter 1 Torque Current", motorShooter1.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Current Shooter 2 Torque Current", motorShooter2.getTorqueCurrent().getValueAsDouble());
  }
  
}
