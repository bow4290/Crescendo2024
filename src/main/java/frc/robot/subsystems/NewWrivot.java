package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;

public class NewWrivot extends SubsystemBase {

  // - Subsystem Constants -
  public static final int MOTOR_ID_WRIST = 15;
  public static final int MOTOR_ID_PIVOT_1 = 13;
  public static final int MOTOR_ID_PIVOT_2 = 14;

  public static final double GEAR_RATIO_PIVOT = 72.96/1;
  public static final double GEAR_RATIO_WRIST = 18.75/1;

  public static final double FEEDFORWARD_PIVOT = 0.1;

  public static final double UPWARD_PIVOT_SPEED_SCALE = 0.35;
  public static final double DOWNWARD_PIVOT_SPEED_SCALE = 0.2;

  public static final double TOLERANCE_PIVOT = 0.1;
  public static final double TOLERANCE_WRIST = 0.2; 

  final TalonFX motorPivot1 = new TalonFX(MOTOR_ID_PIVOT_1);
  final TalonFX motorPivot2 = new TalonFX(MOTOR_ID_PIVOT_2);
  final TalonFX motorWrist = new TalonFX(MOTOR_ID_WRIST);
  
  final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  public BotStates currentBotState = BotStates.INBETWEEN;

  public NewWrivot(){
    // - Pivot Configuration -
    TalonFXConfiguration configurationPivot = new TalonFXConfiguration();

    configurationPivot.Audio.BeepOnBoot = true;
    
    configurationPivot.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configurationPivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configurationPivot.MotorOutput.PeakForwardDutyCycle = 0.5;
    configurationPivot.MotorOutput.PeakReverseDutyCycle = -0.3;
    
    configurationPivot.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.4;

    configurationPivot.Slot0.kP = 12;
    configurationPivot.Slot0.kI = 0;
    configurationPivot.Slot0.kD = 0;

    motorPivot1.getConfigurator().apply(configurationPivot);
    motorPivot2.getConfigurator().apply(configurationPivot);
    motorPivot1.setPosition(0);
    motorPivot2.setControl(new Follower(motorPivot1.getDeviceID(), false));

    // - Wrist Configuration -
    TalonFXConfiguration configurationWrist = new TalonFXConfiguration();

    configurationWrist.Audio.BeepOnBoot = true;

    configurationWrist.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configurationWrist.MotorOutput.PeakForwardDutyCycle = 0.5;
    configurationWrist.MotorOutput.PeakReverseDutyCycle = -0.3;

    configurationWrist.Slot0.kP = 0.35;
    configurationWrist.Slot0.kI = 0;
    configurationWrist.Slot0.kD = 0;

    motorWrist.getConfigurator().apply(configurationWrist);
    motorWrist.setPosition(0);

  }
  

  /**
   * The enum used to store what state the robot is currently in, as well as the target positions for a given state. 
   */
  public enum BotStates {

    // For both wrist and pivot, 0 is at the mechanical stop (pivot all the way down, wrist stashed against mech stop)
    STASH(0, 0),
    INTAKE(0, 130),
    SPEAKER(50, 0),
    AMP(60, 0),
    INBETWEEN(0, 0),
    AIMING(0, 0);

    private double pivotPosition, wristPosition;

    BotStates(double statePivotPosition, double stateWristPosition){
      this.pivotPosition = statePivotPosition;
      this.wristPosition = stateWristPosition;
    }
    public double getPivotDeg(){
      return pivotPosition;
    }
    public double getWristDeg(){
      return wristPosition;
    }

  }

  /**
   * The main command used to go to a desired state. This works by sequencing {@link #cmdGoToDegree()} calls.
   * 
   * @param targetState The desired robot state. Value of type {@link BotStates}
   * @return A {@link SequentialCommandGroup} with the sequenced commands.
   */
  public Command cmdGoToState(BotStates targetState){
     SequentialCommandGroup cmdGroup = new SequentialCommandGroup(
      // Set to inbetween while moving
      new InstantCommand(() -> {currentBotState = BotStates.INBETWEEN;}),
      cmdGoToDegree(motorWrist, BotStates.STASH.getWristDeg(), GEAR_RATIO_WRIST, 0).until(() -> isWristFinished(BotStates.STASH)),
      cmdGoToDegree(motorPivot1, targetState.getPivotDeg(), GEAR_RATIO_PIVOT, 0).until(() -> isPivotFinished(targetState)),
      cmdGoToDegree(motorWrist, targetState.getWristDeg(), GEAR_RATIO_WRIST, 0).until(() -> isWristFinished(targetState)),
      // once everything is known to be finished, update current state to be correct
      new InstantCommand(() -> {currentBotState = targetState;})
     );

     return cmdGroup;
  }

  public Command cmdDrivePivot(double speed){
    
    return this.runEnd(() -> {
      motorPivot1.setControl(dutyCycleRequest.withOutput(speed));
    }, () -> {
      motorPivot1.stopMotor();
      motorPivot2.stopMotor();
    });

  }

  public Command cmdStopPivot(){
    return this.runOnce(() -> {
      motorPivot1.stopMotor();
      motorPivot2.stopMotor();
    });
  }

  public void endMotorRequests(){
    motorPivot1.stopMotor();
    motorPivot2.stopMotor();
    motorWrist.stopMotor();
  }

  private Command cmdGoToDegree(TalonFX targetMotor, double targetDegrees, double gearRatio, double feedForward){
    double positionRotations = Conversions.degToRotationsGearRatio(targetDegrees, gearRatio);

    return this.startEnd(() -> {
      targetMotor.setControl(positionRequest.withPosition(positionRotations).withFeedForward(feedForward));
    }, 
    () -> {
      System.out.println(String.format("cmdGoToDegree finished with target: %f deg", targetDegrees));
    });
  }

  /**
   * Utility used to check whether or not the pivot has finished.
   * @param targetState The targeted state. Value of type BotStates
   * @return Returns whether or not the pivot has finished. Value of type boolean
   */
  private boolean isPivotFinished(BotStates targetState){
    double targetRotations = Conversions.degToRotationsGearRatio(targetState.getPivotDeg(), GEAR_RATIO_PIVOT);

    return MathUtil.isNear(targetRotations, motorPivot1.getPosition().getValueAsDouble(), TOLERANCE_PIVOT);
  }

    /**
   * Utility used to check whether or not the wrist has finished.
   * @param targetState The targeted state. Value of type BotStates
   * @return Returns whether or not the wrist has finished. Value of type boolean
   */
  private boolean isWristFinished(BotStates targetState){
    double targetRotations = Conversions.degToRotationsGearRatio(targetState.getWristDeg(), GEAR_RATIO_WRIST);
    
    return MathUtil.isNear(targetRotations, motorWrist.getPosition().getValueAsDouble(), TOLERANCE_WRIST);
  }


  @Override
  public void periodic(){
    SmartDashboard.putString("Robot State", currentBotState.toString());
    
    SmartDashboard.putNumber("Pivot Degrees", (motorPivot1.getPosition().getValueAsDouble() * 360) / GEAR_RATIO_PIVOT);
    SmartDashboard.putNumber("Wrist Degrees", (motorWrist.getPosition().getValueAsDouble() * 360) / GEAR_RATIO_WRIST);

    SmartDashboard.putNumber("Current Pivot Motor Rotations", motorPivot1.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Current Wrist Motor Rotations", motorWrist.getPosition().getValueAsDouble());
  }

}
