package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.math.Conversions;

public class WrivotStates extends SubsystemBase {
    // - Subystem Constants -
    public static final int MOTOR_ID_WRIST = 0;
    public static final int MOTOR_ID_PIVOT_1 = 13;
    public static final int MOTOR_ID_PIVOT_2 = 14;

    public static final double GEAR_RATIO_PIVOT = 128/1;
    public static final double GEAR_RATIO_WRIST = 64/1; // TODO: find proper value of this from dylan 

    // Other Declarations
    private final TalonFX motorWrist = new TalonFX(MOTOR_ID_WRIST);
    private final TalonFX motorPivot1 = new TalonFX(MOTOR_ID_PIVOT_1);
    private final TalonFX motorPivot2 = new TalonFX(MOTOR_ID_PIVOT_2); // Follower of motorPivot1

    final PositionVoltage requestPositionVoltage = new PositionVoltage(0).withSlot(0);

    private BotAngleState currentState = BotAngleState.INTERMEDIATE;

    public WrivotStates(){
        motorPivot2.setControl(new Follower(MOTOR_ID_PIVOT_1, false));

        // - Pivot Configuration -
        TalonFXConfiguration configurationPivot = new TalonFXConfiguration();

        configurationPivot.Audio.BeepOnBoot = true;
        
        configurationPivot.Slot0.kP = 12; // TODO: tune pivot PID
        configurationPivot.Slot0.kI = 0;
        configurationPivot.Slot0.kD = 0.1;

        motorPivot1.getConfigurator().apply(configurationPivot);

        // - Wrist Configuration -
        TalonFXConfiguration configurationWrist = new TalonFXConfiguration();

        configurationWrist.Audio.BeepOnBoot = true;

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
    public enum BotAngleState {
        STASH(0, 0),
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
     * A utility function meant to be used exclusively inside {@link WrivotStates}. Runs a closed-loop control request for the given motor.
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
