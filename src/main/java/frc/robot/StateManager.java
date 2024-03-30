package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * A class made to store all of our interpolation tables, using {@link InterpolatingDoubleTreeMap}.
 */
public class StateManager {

  /**
   * An enum used to store current bot state, as well as target positions for both pivot and wrist in that state (in degrees).
   * 
   * @apiNote (pivotDegrees, wristDegrees)
   */
  public enum BotState {
    STASH(0, 0),
    INTAKE(0, 0),
    SPEAKER_BASE(0, 0),
    SPEAKER_AIMING(0, 0),
    AMP(0, 0),
    INBETWEEN(0, 0);

    public final double speakerDegrees;
    public final double wristDegrees;

    private BotState(double speakerDegrees, double wristDegrees){
      this.speakerDegrees = speakerDegrees;
      this.wristDegrees = wristDegrees;
    }
  }

  /**
   * An interpolation table for getting the angles at which wrist can score speaker from a given distance. This is based on the pivot being at the {@link BotState#SPEAKER_BASE} state.
   * 
   * @apiNote Key: Distance
   * @apiNote Value: Angle
   */
  public static InterpolatingDoubleTreeMap speakerWristDegTable = new InterpolatingDoubleTreeMap();

  static {
    
  }
}
