package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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

  public static final double CLIMBER_UP_SPEED = 0.4;
  public static final double CLIMBER_DOWN_SPEED = 0.6;

  final TalonFX motorClimber1 = new TalonFX(MOTOR_ID_CLIMBER_1);
  final TalonFX motorClimber2 = new TalonFX(MOTOR_ID_CLIMBER_2);

  final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public Climber(){
    TalonFXConfiguration climberConfiguration = new TalonFXConfiguration();
    
    climberConfiguration.Audio.BeepOnBoot = true;
    
    climberConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    motorClimber1.getConfigurator().apply(climberConfiguration);
    motorClimber2.getConfigurator().apply(climberConfiguration);
    
    motorClimber1.setInverted(true);
  }

  public Command cmdClimbSide(double targetOut, TalonFX targetMotor, WrivotStates reqWrivotStates){

    StartEndCommand cmd = new StartEndCommand(() -> {
      targetMotor.setControl(dutyCycleOut.withOutput(targetOut));
    }, 
    () -> {
      targetMotor.stopMotor();
    }, this, reqWrivotStates);

    return cmd;
  }

  public Command cmdClimbTogether(double targetOut, WrivotStates reqWrivotStates){

    StartEndCommand cmd = new StartEndCommand(() -> {
      motorClimber1.setControl(dutyCycleOut.withOutput(targetOut));
      motorClimber2.setControl(dutyCycleOut.withOutput(targetOut));
    }, 
    () -> {
      motorClimber1.stopMotor();
      motorClimber2.stopMotor();
    }, this, reqWrivotStates);

    return cmd;
  }
  
}
