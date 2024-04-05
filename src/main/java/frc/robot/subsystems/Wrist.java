package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;

public class Wrist extends SubsystemBase {

  public static final int MOTOR_ID_WRIST = 15;

  public static final double GEAR_RATIO_WRIST = 27.5/1;

  // Wrist zero is set to be more horizontal so that gravity feedforward works, making this the mech stop offset
  public static final double MECH_STOP_OFFSET = 0.309;

  public static final double TOLERANCE = 0.05; 

  final TalonFX motorWrist = new TalonFX(MOTOR_ID_WRIST);

  final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  public Wrist() {
    TalonFXConfiguration configurationWrist = new TalonFXConfiguration();
    
    configurationWrist.Audio.BeepOnBoot = true;

    configurationWrist.Feedback.SensorToMechanismRatio = GEAR_RATIO_WRIST;

    configurationWrist.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configurationWrist.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configurationWrist.MotorOutput.PeakForwardDutyCycle = 0.5;
    configurationWrist.MotorOutput.PeakReverseDutyCycle = -0.3;

    // TODO: Tune these values (all)
    configurationWrist.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configurationWrist.Slot0.kG = 0.95;
    configurationWrist.Slot0.kS = 1.8;
    configurationWrist.Slot0.kV = 0.12;
    configurationWrist.Slot0.kA = 0.1;
    configurationWrist.Slot0.kP = 20;
    configurationWrist.Slot0.kI = 0;
    configurationWrist.Slot0.kD = 4.5;

    configurationWrist.MotionMagic.MotionMagicCruiseVelocity = 1;
    configurationWrist.MotionMagic.MotionMagicAcceleration = 3;

    motorWrist.getConfigurator().apply(configurationWrist);
    motorWrist.setPosition(MECH_STOP_OFFSET);
  }

  public Command cmdWristToDeg(double targetDegrees){
    double targetRotations = Conversions.degToRotations(targetDegrees);

    return this.startEnd(() -> {
      motorWrist.setControl(motionMagicVoltage.withSlot(0).withPosition(targetRotations));
    }, () -> {});
  }

  public boolean isWristFinished(double targetDegrees){
    double targetRotations = Conversions.degToRotations(targetDegrees);
    
    return MathUtil.isNear(targetRotations, motorWrist.getPosition().getValueAsDouble(), TOLERANCE);
  }

  public void wristStop(){
    motorWrist.stopMotor();
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Wrist Rotations (With GR)", motorWrist.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Wrist Rotations (Raw)", motorWrist.getRotorPosition().getValueAsDouble());
    SmartDashboard.putNumber("Wrist Deg (Horizontal Zero)", motorWrist.getPosition().getValueAsDouble() * 360);
  }



}
