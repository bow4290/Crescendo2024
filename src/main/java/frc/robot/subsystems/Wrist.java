package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.subsystems.OldSubsystems.NewWrivot.BotStates;

public class Wrist extends SubsystemBase {

  public static final int MOTOR_ID_WRIST = 15;

  public static final double GEAR_RATIO_WRIST = 18.75/1;

  public static final double TOLERANCE = 0.2; 

  final TalonFX motorWrist = new TalonFX(MOTOR_ID_WRIST);

  final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  public Wrist() {
    TalonFXConfiguration configurationWrist = new TalonFXConfiguration();
    
    configurationWrist.Audio.BeepOnBoot = true;

    configurationWrist.Feedback.SensorToMechanismRatio = GEAR_RATIO_WRIST;

    configurationWrist.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configurationWrist.MotorOutput.PeakForwardDutyCycle = 0.5;
    configurationWrist.MotorOutput.PeakReverseDutyCycle = -0.3;

    // TODO: Tune these values (all)
    configurationWrist.Slot0.kS = 0;
    configurationWrist.Slot0.kV = 0;
    configurationWrist.Slot0.kA = 0;
    configurationWrist.Slot0.kP = 0.35;
    configurationWrist.Slot0.kI = 0;
    configurationWrist.Slot0.kD = 0;

    configurationWrist.MotionMagic.MotionMagicCruiseVelocity = 10;
    configurationWrist.MotionMagic.MotionMagicAcceleration = 20;
    configurationWrist.MotionMagic.MotionMagicJerk = 200;

    motorWrist.getConfigurator().apply(configurationWrist);
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



}
