package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;

public class Pivot extends SubsystemBase {

  public static final int MOTOR_ID_PIVOT_1 = 13;
  public static final int MOTOR_ID_PIVOT_2 = 14;

  public static final double GEAR_RATIO_PIVOT = 72.96/1;

  public static final double UPWARD_PIVOT_SPEED_SCALE = 0.35;
  public static final double DOWNWARD_PIVOT_SPEED_SCALE = 0.2;
  
  public static final double TOLERANCE = 0.3;

  final TalonFX motorPivot1 = new TalonFX(MOTOR_ID_PIVOT_1);
  final TalonFX motorPivot2 = new TalonFX(MOTOR_ID_PIVOT_2);

  final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  public Pivot(){
    TalonFXConfiguration configurationPivot = new TalonFXConfiguration();

    configurationPivot.Audio.BeepOnBoot = true;

    configurationPivot.Feedback.SensorToMechanismRatio = GEAR_RATIO_PIVOT;

    configurationPivot.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configurationPivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configurationPivot.MotorOutput.PeakForwardDutyCycle = 0.5;
    configurationPivot.MotorOutput.PeakReverseDutyCycle = -0.3;

    configurationPivot.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.4;

    // TODO: Retune these values with redesign
    configurationPivot.Slot0.kS = 0;
    configurationPivot.Slot0.kV = 0;
    configurationPivot.Slot0.kA = 0;
    configurationPivot.Slot0.kP = 1;
    configurationPivot.Slot0.kI = 0;
    configurationPivot.Slot0.kD = 0;

    // TODO: Figure out what these should be? (jerk is a dumb measurement)
    configurationPivot.MotionMagic.MotionMagicCruiseVelocity = 40;
    configurationPivot.MotionMagic.MotionMagicAcceleration = 160;
    configurationPivot.MotionMagic.MotionMagicJerk = 1600;

    motorPivot1.getConfigurator().apply(configurationPivot);
    motorPivot2.getConfigurator().apply(configurationPivot);
    motorPivot1.setPosition(0);
    motorPivot2.setControl(new Follower(motorPivot1.getDeviceID(), false));
  }

  public Command cmdPivotToDeg(double targetDegrees){
    double targetRotations = Conversions.degToRotations(targetDegrees);

    return this.startEnd(() -> {
      motorPivot1.setControl(motionMagicVoltage.withSlot(0).withPosition(targetRotations));
    }, () -> {

    });
  }

  public boolean isPivotFinished(double targetDegrees){
    double targetRotations = Conversions.degToRotations(targetDegrees);

    return MathUtil.isNear(targetRotations, motorPivot1.getPosition().getValueAsDouble(), TOLERANCE);
  }

  public void pivotStop(){
    motorPivot1.stopMotor();
    motorPivot2.stopMotor();
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Pivot Rotations (With GR)", motorPivot1.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Rotations (Raw)", motorPivot1.getRotorPosition().getValueAsDouble());
  }
  
}
