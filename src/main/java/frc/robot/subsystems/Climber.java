package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

  public static final double CLIMBER_UP_SPEED_RPM = 90;
  public static final double CLIMBER_DOWN_SPEED_RPM = 70;

  final TalonFX motorClimber1 = new TalonFX(MOTOR_ID_CLIMBER_1);
  final TalonFX motorClimber2 = new TalonFX(MOTOR_ID_CLIMBER_2);

  final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public Climber(){
    TalonFXConfiguration climberConfiguration = new TalonFXConfiguration();
    
    climberConfiguration.Audio.BeepOnBoot = true;
    
    climberConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    climberConfiguration.Slot0.kS = 0.05;
    climberConfiguration.Slot0.kV = 0.12;
    climberConfiguration.Slot0.kP = 0.11;
    climberConfiguration.Slot0.kI = 0.0;
    climberConfiguration.Slot0.kD = 0.0;
    
    motorClimber1.getConfigurator().apply(climberConfiguration);
    motorClimber2.getConfigurator().apply(climberConfiguration);
    
    motorClimber1.setInverted(true);
  }

  public Command cmdClimbSide(double velocityRPM, TalonFX targetMotor, WrivotStates reqWrivotStates){
    double velocityRPS = (velocityRPM * 60) * GEAR_RATIO_CLIMBER;
    
    StartEndCommand cmd = new StartEndCommand(() -> {
      targetMotor.setControl(velocityRequest.withVelocity(velocityRPS));
    }, 
    () -> {
      targetMotor.stopMotor();
    }, this, reqWrivotStates);

    return cmd;
  }

  public Command cmdClimbTogether(double velocityRPM, WrivotStates reqWrivotStates){
    double velocityRPS = (velocityRPM * 60) * GEAR_RATIO_CLIMBER;

    StartEndCommand cmd = new StartEndCommand(() -> {
      motorClimber1.setControl(velocityRequest.withVelocity(velocityRPS));
      motorClimber2.setControl(velocityRequest.withVelocity(velocityRPS));
    }, 
    () -> {
      motorClimber1.stopMotor();
      motorClimber2.stopMotor();
    }, this, reqWrivotStates);

    return cmd;
  }


  
}
