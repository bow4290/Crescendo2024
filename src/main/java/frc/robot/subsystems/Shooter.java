package frc.robot.subsystems;


//he he he

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;

public class Shooter extends SubsystemBase {
  // Subsystem Constants
  public static final int MOTOR_ID_SHOOTER = 10;
  public static final int MOTOR_ID_INDEXER = 16;

  public static final double SHOOTER_IN_RPM = -4000; // TODO: adjust Shooter in Speed to be more accurate
  public static final double SHOOTER_OUT_RPM = 6000;
  public static final double SHOOTER_PRESPIN_RPM = 1500;
  
  public static final double INDEXER_IN_SPEED = -0.45; // TODO: adjust Indexer in Speed to be more accurate
  public static final double INDEXER_OUT_SPEED = 0.75; // TODO: adjust Indexer out Speed to be more accurate

  public static final double GEAR_RATIO_SHOOTER = 1/1.8;

  private TalonFX motorShooter = new TalonFX(MOTOR_ID_SHOOTER);
  private TalonFX motorIndexer = new TalonFX(MOTOR_ID_INDEXER);

  final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  final VelocityVoltage velocityOut = new VelocityVoltage(0).withSlot(0);

public Shooter(){
  TalonFXConfiguration configurationShooter = new TalonFXConfiguration();
  configurationShooter.Slot0.kS = 0.05;
  configurationShooter.Slot0.kV = 0.12;
  configurationShooter.Slot0.kP = 2.1;
  configurationShooter.Slot0.kI = 0.0;
  configurationShooter.Slot0.kD = 0;

  motorShooter.getConfigurator().apply(configurationShooter);
}

  // - Public Commands -
  public Command cmdShootOut(){
    return Commands.sequence(
      cmdStartShooter(SHOOTER_OUT_RPM),
      Commands.waitUntil(() -> isShooterSpeed(SHOOTER_OUT_RPM)).withTimeout(2.5),
      cmdRunIndexer(INDEXER_OUT_SPEED)
    ).handleInterrupt(this::stopShooter);
  }

  public Command cmdShooterIntake(){
    return Commands.sequence(
      cmdStartShooter(SHOOTER_IN_RPM),
      cmdRunIndexer(INDEXER_IN_SPEED)
    ).handleInterrupt(this::stopShooter);
  }

  // - Internal Commands -
  private Command cmdRunIndexer(double targetSpeed){
    return this.startEnd(
    () -> {
      motorIndexer.setControl(dutyCycleOut.withOutput(targetSpeed));
    },
    () -> {
      motorIndexer.stopMotor();
    });
  }

  public Command cmdStartShooter(double targetRPM){
    double targetRPS = Conversions.rpmToRpsGearRatio(targetRPM, GEAR_RATIO_SHOOTER);

    return this.runOnce(
    () -> {
      motorShooter.setControl(velocityOut.withVelocity(targetRPS));
    });
  }

  public Command cmdStopShooter(){
    return this.runOnce(() -> stopShooter());
  }

  private void stopShooter(){
    motorShooter.stopMotor();
  }

  public boolean isShooterSpeed(double targetRPM){
    double targetVelocity = targetRPM * GEAR_RATIO_SHOOTER;
    if (Math.abs(motorShooter.getVelocity().getValueAsDouble() * 60 - targetVelocity) <= 10){
      return true;
    }else {
      return false;
    }
  }

  public void endMotorRequests(){
    motorIndexer.stopMotor();
    motorShooter.stopMotor();
  }


  @Override
  public void periodic(){
    SmartDashboard.putNumber("Current Shooter Speed", motorShooter.get());
    SmartDashboard.putNumber("Current Shooter Velocity", motorShooter.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("Current Indexer Speed", motorIndexer.get());
    SmartDashboard.putNumber("Current Index Velocity", motorIndexer.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("Current Shooter Velocity (RPM)", motorShooter.getVelocity().getValueAsDouble() * 60);

  }
    
}
































//I see you've found me