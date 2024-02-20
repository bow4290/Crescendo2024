package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.math.Conversions;

public class WrivotStates extends SubsystemBase {
    // - Subystem Constants -
    public static final int MOTOR_ID_WRIST = 15;
    public static final int MOTOR_ID_PIVOT_1 = 13;
    public static final int MOTOR_ID_PIVOT_2 = 14;
    public static final int ENCODER_ID_PIVOT = 0;
    public static final int ENCODER_ID_WRIST = 1;


    public static final double GEAR_RATIO_PIVOT = 73/1;
    public static final double GEAR_RATIO_WRIST = 32/1;

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
        motorPivot1.setPosition(encoderPivot.get() * GEAR_RATIO_PIVOT);
        motorWrist.setPosition(encoderWrist.get() * GEAR_RATIO_WRIST);

        // - Pivot Configuration -
        TalonFXConfiguration configurationPivot = new TalonFXConfiguration();

        configurationPivot.Audio.BeepOnBoot = true;

        configurationPivot.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configurationPivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        configurationPivot.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.6;
        
        configurationPivot.Slot0.kP = 12; // TODO: tune pivot PID
        configurationPivot.Slot0.kI = 0;
        configurationPivot.Slot0.kD = 0.1;

        motorPivot1.getConfigurator().apply(configurationPivot);

        // - Wrist Configuration -
        TalonFXConfiguration configurationWrist = new TalonFXConfiguration();

        configurationWrist.Audio.BeepOnBoot = true;

        configurationWrist.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        configurationWrist.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.8;

        configurationWrist.Slot0.kP = 12; // TODO: tune wrist PID
        configurationWrist.Slot0.kI = 0;
        configurationWrist.Slot0.kD = 0.1;

        motorWrist.getConfigurator().apply(configurationWrist);
    }

    /**
     * The enum used to manage state in the robot. 
     * Each constant state has two parameters, pivotDegrees, and wristDegrees, both of which are double values for the intended angle. <br>
     * These parameters can be accessed with the getPivotDegrees() and getWristDegrees(). 
     */
    public enum BotAngleState { // TODO: TUNE / INPUT VALUES!! VERY IMPORTANT.
        STASH(-10, 0),
        INTAKE(0, 0),
        SPEAKER(0, 0),
        AMP(0, 0),
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

    /**
     * The main utility function for exclusive use inside {@link WrivotStates}, in commands. Sequences using goToDegree().
     * 
     * @apiNote Sequence order:  Wrist (Stash) -> Pivot (Target) -> Wrist (Target)
     * 
     * @param targetPivotDegrees Pivot target position (in degrees). Value of type double.
     * @param targetWristDegrees Wrist target position (in degrees). Value of type double.
     */
    private void wrivotSequencer(double targetPivotDegrees, double targetWristDegrees){
        // Make sure wrist is stashed before running anything else
        goToDegree(motorWrist, BotAngleState.STASH.getWristDegrees(), GEAR_RATIO_WRIST);
        // Pivot to Target
        goToDegree(motorPivot1, targetPivotDegrees, GEAR_RATIO_PIVOT);
        // Wrist to Target
        goToDegree(motorWrist, targetWristDegrees, GEAR_RATIO_WRIST);
    }

    /**
     * A utility function meant to be used exclusively inside {@link WrivotStates}, specifically inside the wrivotSequencer(). Runs a closed-loop control request for the given motor.
     * PID should be tuned in the motor before passing it.
     * 
     * @param talonMotor The motor to set the control. Value of type {@link TalonFX}.
     * @param degrees The degrees to go to. Value of type double.
     * @param gearRatio The gear ratio of the hardware to interact with. Value of type double.
     */
    private void goToDegree(TalonFX talonMotor, double degrees, double gearRatio){
        double setPos = Conversions.degToRotationsGearRatio(degrees, gearRatio);
        talonMotor.setControl(requestPositionVoltage.withPosition(setPos));
    }




}