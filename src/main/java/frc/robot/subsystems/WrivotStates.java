package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WrivotStates extends SubsystemBase {
    // - Subystem Constants -
    public static final int MOTOR_ID_WRIST = 0;
    public static final int MOTOR_ID_PIVOT_1 = 13;
    public static final int MOTOR_ID_PIVOT_2 = 14;

    // Other Declarations
    private TalonFX motorWrist = new TalonFX(MOTOR_ID_WRIST);
    private TalonFX motorPivot1 = new TalonFX(MOTOR_ID_PIVOT_1);
    private TalonFX motorPivot2 = new TalonFX(MOTOR_ID_PIVOT_2);

    private BotAngleState currentState = BotAngleState.INTERMEDIATE;

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

    


}
