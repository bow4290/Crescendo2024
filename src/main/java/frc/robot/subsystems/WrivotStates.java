package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.math.Conversions;

public class WrivotStates extends SubsystemBase {
    // - Subystem Constants -
    public static final int MOTOR_ID_WRIST = 15;
    public static final int MOTOR_ID_PIVOT_1 = 13;
    public static final int MOTOR_ID_PIVOT_2 = 14;
    public static final int ENCODER_ID_PIVOT = 0;
    public static final int ENCODER_ID_WRIST = 8;

    public static final double GEAR_RATIO_PIVOT = 72.96/1;
    public static final double GEAR_RATIO_WRIST = 18.75/1;

    // Use these to get the actual zero of a through bore encoder, because 0 on it is often not what we want as zero. 
    // Positive if the value is negative at "zero", negative if value is positive at "zero"
    public static final double ENCODER_OFFSET_PIVOT = 0;
    public static final double ENCODER_OFFSET_WRIST = -0.258;

    public static final double TOLERANCE = 0.01; 

    // Other Declarations
    private final TalonFX motorWrist = new TalonFX(MOTOR_ID_WRIST);
    private final TalonFX motorPivot1 = new TalonFX(MOTOR_ID_PIVOT_1);
    private final TalonFX motorPivot2 = new TalonFX(MOTOR_ID_PIVOT_2); // Follower of motorPivot1
    private final DutyCycleEncoder encoderPivot = new DutyCycleEncoder(ENCODER_ID_PIVOT);
    private final DutyCycleEncoder encoderWrist = new DutyCycleEncoder(ENCODER_ID_WRIST);

    final PositionVoltage requestPositionVoltage = new PositionVoltage(0).withSlot(0);

    private BotAngleState currentState = BotAngleState.INTERMEDIATE;
    
    public WrivotStates(){
        motorPivot2.setControl(new Follower(MOTOR_ID_PIVOT_1, false));

        // Get the absolute encoder position on startup, and set the motors position to it. 
        // maybe make this into getAbsolutePosition? I think its for how many turns, we dont need that.
        motorPivot1.setPosition(getEncoderWithOffset(encoderPivot.getAbsolutePosition(), ENCODER_OFFSET_PIVOT) * GEAR_RATIO_PIVOT);
        motorWrist.setPosition(getEncoderWithOffset(encoderWrist.getAbsolutePosition(), ENCODER_OFFSET_WRIST) * GEAR_RATIO_WRIST);

        // - Pivot Configuration -
        TalonFXConfiguration configurationPivot = new TalonFXConfiguration();

        configurationPivot.Audio.BeepOnBoot = true;

        configurationPivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configurationPivot.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        configurationPivot.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.2;
        configurationPivot.MotorOutput.PeakReverseDutyCycle = 0.5;
        
        configurationPivot.Slot0.kP = 1.5; // TODO: tune pivot PID
        configurationPivot.Slot0.kI = 0.0;
        configurationPivot.Slot0.kD = 0.0;

        motorPivot1.getConfigurator().apply(configurationPivot);
        motorPivot2.getConfigurator().apply(configurationPivot);

        // - Wrist Configuration -
        TalonFXConfiguration configurationWrist = new TalonFXConfiguration();

        configurationWrist.Audio.BeepOnBoot = true;

        configurationWrist.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        configurationWrist.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.2;
        configurationWrist.MotorOutput.PeakForwardDutyCycle = 0.5;
        configurationWrist.MotorOutput.PeakReverseDutyCycle = 0.3;

        configurationWrist.Slot0.kP = 0.15; // TODO: tune wrist PID
        configurationWrist.Slot0.kI = 0.0;
        configurationWrist.Slot0.kD = 0.0;

        motorWrist.getConfigurator().apply(configurationWrist);
    }

    /**
     * The enum used to manage state in the robot. 
     * Each constant state has two parameters, pivotDegrees, and wristDegrees, both of which are double values for the intended angle. <br>
     * These parameters can be accessed with the getPivotDegrees() and getWristDegrees(). 
     */
    public enum BotAngleState { // TODO: TUNE / INPUT VALUES!! VERY IMPORTANT.
        STASH(-7.5, 0),
        INTAKE(-7.5, 140),
        SPEAKER(30, 0),
        AMP(65, 0),
        AIMING(0, 0){
            @Override
            public double getPivotDegrees() {
                System.err.println("getPivotDegrees() called on AIMING state, returning 404 error");
                return 404; // Error / don't use!
            }
        },
        INTERMEDIATE(0, 0){
            @Override
            public double getPivotDegrees() {
                System.err.println("getPivotDegrees() called on INTERMEDIATE state, returning 404 error");
                return 404;
            }

            @Override
            public double getWristDegrees(){
                System.err.println("getWristDegrees() called on INTERMEDIATE state, returning 404 error");
                return 404;
            }
        };

        private final double pivotDegrees;
        private final double wristDegrees;

        /**
         * @param pivotDegrees The degrees for pivot at a given state. Value of type double.
         * @param wristDegrees The degrees for wrist at a given state. Value of type double
         */
        BotAngleState(double pivotDegrees, double wristDegrees){
            this.pivotDegrees = pivotDegrees;
            this.wristDegrees = wristDegrees;
        }

        public double getPivotDegrees(){
            return pivotDegrees;
        }

        public double getWristDegrees(){
            return wristDegrees;
        }

    }
    public BotAngleState getCurrentState(){
      return currentState;
    }

    // start of non-enum code (checkpoint)

    public Command cmdWrivotSequencer(BotAngleState targetState){
      double targetPivotDeg = targetState.getPivotDegrees();
      double targetWristDeg = targetState.getWristDegrees();

      // return 
      //   cmdGoToDegree(motorWrist, BotAngleState.STASH.getWristDegrees(), GEAR_RATIO_WRIST).until(() -> isWristFinished(BotAngleState.STASH))
      //   .andThen(cmdGoToDegree(motorPivot1, targetPivotDeg, GEAR_RATIO_PIVOT)).until(() -> isPivotFinished(targetState))
      //   .andThen(cmdGoToDegree(motorWrist, targetWristDeg, GEAR_RATIO_WRIST)).until(() -> isWristFinished(targetState));

      return cmdGoToDegree(motorPivot1, targetPivotDeg, GEAR_RATIO_PIVOT).until(() -> isPivotFinished(targetState));
    }

    private Command cmdGoToDegree(TalonFX setMotor, double degrees, double gearRatio){
      return this.startEnd(() -> {
        double targetRotations = Conversions.degToRotationsGearRatio(degrees, gearRatio);
        setMotor.setControl(requestPositionVoltage.withPosition(targetRotations));
        System.err.println("cmdGoToDegree called");
        System.err.flush();
      }, () -> {});
    }


    /**
     * Gets the encoder value with the offset AND clipped / corrected rotations.
     * 
     * @param rawValue Rotations before offset. Value of type double.
     * @param offsetValue Rotations you apply offset. Value of type double.
     * @return Rotations with offset applied as well as corrected clipping.
     */
    private double getEncoderWithOffset(double rawValue, double offsetValue){
      double value = Math.floor(rawValue * 1000) / 1000;
      
      return getCorrectedRotations(value + offsetValue);
    }

    private double getCorrectedRotations(double inputRotations){
      double tempDegAngle = 360 * inputRotations;
      if (tempDegAngle < -90){
        return (360 + tempDegAngle) / 360;
      } else {
        return inputRotations;
      }
    }

    private boolean isPivotFinished(BotAngleState targetState){
      double targetRotations = Conversions.degToRotations(targetState.getPivotDegrees());
      boolean returnValue;

      System.err.println(String.format("Pivot Target Rotations: %f", targetRotations));
      System.err.flush();

      if (Math.abs(targetRotations - getEncoderWithOffset(encoderPivot.getAbsolutePosition(), ENCODER_OFFSET_PIVOT)) <= TOLERANCE){
        returnValue = true;
        System.err.println("Pivot Finished");
        System.err.flush();
      }else{
        returnValue = false;
      }

      return returnValue;
    }

    private boolean isWristFinished(BotAngleState targetState){
      double targetRotations = Conversions.degToRotations(targetState.getWristDegrees());
      boolean returnValue;

      System.err.println(String.format("Wrist Target Rotations: %f", targetRotations));
      System.err.flush();

      if (Math.abs(targetRotations - getEncoderWithOffset(encoderWrist.getAbsolutePosition(), ENCODER_OFFSET_WRIST)) <= TOLERANCE){
        returnValue = true;
        System.err.println("Wrist Finished");
        System.err.flush();
      }else{
        returnValue = false;
      }

      return returnValue;
    }

    public void endMotorRequests(){
      motorPivot1.stopMotor();
      motorPivot2.stopMotor();
      motorWrist.stopMotor();
    }

    @Override
    public void periodic(){

      SmartDashboard.putNumber("Pivot Motor Speed", motorPivot1.get());
      SmartDashboard.putNumber("Wrist Motor Speed", motorWrist.get());
      
      SmartDashboard.putNumber("Pivot Encoder Position", getEncoderWithOffset(encoderPivot.getAbsolutePosition(), ENCODER_OFFSET_PIVOT));
      SmartDashboard.putNumber("Wrist Encoder Position", getEncoderWithOffset(encoderWrist.getAbsolutePosition(), ENCODER_OFFSET_WRIST));

      SmartDashboard.putString("Current Robot State", getCurrentState().toString());

      // Debug Values
      SmartDashboard.putNumber("(Debug) Raw Pivot Encoder Pos", encoderPivot.getAbsolutePosition());
      SmartDashboard.putNumber("(Debug) Raw Wrist Encoder Pos", encoderWrist.getAbsolutePosition());
      SmartDashboard.putNumber("(Debug) Raw Pivot Motor 1 Pos", motorPivot1.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("(Debug) Raw Wrist Motor Pos", motorWrist.getPosition().getValueAsDouble());
    }
    
}
