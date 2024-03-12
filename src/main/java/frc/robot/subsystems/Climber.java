package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;

public class Climber extends SubsystemBase {

  // - Subsystem Constants -
  public static final int MOTOR_ID_CLIMBER_1 = 12;
  public static final int MOTOR_ID_CLIMBER_2 = 11;
  
  public static final double GEAR_RATIO_CLIMBER = 32/1;

  // Negative Up (i know)
  public static final double CLIMBER_UP_SPEED = -0.4;
  public static final double CLIMBER_DOWN_SPEED = 0.6;

  public static final double TOLERANCE = 1.2;

  final TalonFX motorClimber1 = new TalonFX(MOTOR_ID_CLIMBER_1);
  final TalonFX motorClimber2 = new TalonFX(MOTOR_ID_CLIMBER_2);

  final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public Climber(){
    TalonFXConfiguration climberConfiguration = new TalonFXConfiguration();
    
    climberConfiguration.Audio.BeepOnBoot = true;
    
    climberConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climberConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
    climberConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0;

    climberConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    climberConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = -43;

    climberConfiguration.HardwareLimitSwitch.ForwardLimitEnable = true;
    climberConfiguration.HardwareLimitSwitch.ReverseLimitEnable = true;
    
    motorClimber1.getConfigurator().apply(climberConfiguration);
    motorClimber2.getConfigurator().apply(climberConfiguration);

    motorClimber1.setInverted(true);

    motorClimber1.setPosition(0);
    motorClimber2.setPosition(0);
  }

  public Command cmdClimbSide(double targetOut, TalonFX targetMotor){

    StartEndCommand cmd = new StartEndCommand(() -> {
      targetMotor.setControl(dutyCycleOut.withOutput(targetOut));
    }, 
    () -> {
      targetMotor.stopMotor();
    }, this);

    return cmd;
  }

  public Command cmdClimbTogether(double targetOut){

    StartEndCommand cmd = new StartEndCommand(() -> {
      if (!shouldClimberStop(targetOut)) {
        motorClimber1.setControl(dutyCycleOut.withOutput(targetOut));
        motorClimber2.setControl(dutyCycleOut.withOutput(targetOut)); 
      }
    }, 
    () -> {
      motorClimber1.stopMotor();
      motorClimber2.stopMotor();
    }, this);

    return cmd;
  }

  private boolean shouldClimberStop(double targetSpeed){
    double motor1Pos = motorClimber1.getPosition().getValueAsDouble();
    double motor2Pos = motorClimber2.getPosition().getValueAsDouble();

    // Positive (everything is truly awful sometimes)
    if (targetSpeed < 0){
      if (MathUtil.isNear(-43, motor1Pos, TOLERANCE) || MathUtil.isNear(-43, motor2Pos, TOLERANCE)){
        return true;
      }
    } else {
      if (MathUtil.isNear(0, motor1Pos, TOLERANCE) || MathUtil.isNear(0, motor2Pos, TOLERANCE)){
        return true;
      }
    }

    return false;
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Climber 1 Motor Raw Pos", motorClimber1.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climber 2 Motor Raw Pos", motorClimber2.getPosition().getValueAsDouble());
  }  
}
