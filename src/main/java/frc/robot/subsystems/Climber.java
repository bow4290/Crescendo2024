package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  // - Subsystem Constants -
  public static final int MOTOR_ID_CLIMBER_1 = 12;
  public static final int MOTOR_ID_CLIMBER_2 = 11;
  
  public static final double GEAR_RATIO_CLIMBER = 32/1;

  // Negative Up (i know)
  public static final double CLIMBER_UP_SPEED = -0.4;
  public static final double CLIMBER_DOWN_SPEED = 0.6;

  // Negative Up (i know (again))
  public static final double UPPER_LIMIT = -35;
  public static final double LOWER_LIMIT = 0;

  public static final double TOLERANCE = 1;

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
    climberConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = -40;

    climberConfiguration.HardwareLimitSwitch.ForwardLimitEnable = true;
    climberConfiguration.HardwareLimitSwitch.ReverseLimitEnable = true;
    
    motorClimber1.getConfigurator().apply(climberConfiguration);
    motorClimber2.getConfigurator().apply(climberConfiguration);

    motorClimber1.setInverted(true);

    motorClimber1.setPosition(0);
    motorClimber2.setPosition(0);
  }



  public Command cmdClimbTogether(double targetOut){
    ParallelCommandGroup cmd = new ParallelCommandGroup(
      cmdStartSide(targetOut, motorClimber1).handleInterrupt(() -> motorClimber1.stopMotor()).until(() -> shouldSideStop(motorClimber1, targetOut)),
      cmdStartSide(targetOut, motorClimber2).handleInterrupt(() -> motorClimber2.stopMotor()).until(() -> shouldSideStop(motorClimber2, targetOut))
    );

    return cmd;
  }

  private Command cmdStartSide(double targetOut, TalonFX targetMotor){
    StartEndCommand cmd = new StartEndCommand(() -> {
      if (!shouldSideStop(targetMotor, targetOut)){
        targetMotor.setControl(dutyCycleOut.withOutput(targetOut));
      }
    }, 
    () -> {

    });
    return cmd;
  }

  private boolean shouldSideStop(TalonFX targetMotor, double targetSpeed){

    // Positive (everything is truly awful sometimes)
    if (targetSpeed < 0){
      if (MathUtil.isNear(UPPER_LIMIT, targetMotor.getPosition().getValueAsDouble(), TOLERANCE)){
        return true;
      }
    } else {
      if (MathUtil.isNear(LOWER_LIMIT, targetMotor.getPosition().getValueAsDouble(), TOLERANCE)){
        return true;
      }
    }

    return false;
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Climber Motor 1 Pos (Raw)", motorClimber1.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climber Motor 2 Pos (Raw)", motorClimber2.getPosition().getValueAsDouble());
  }  
}
