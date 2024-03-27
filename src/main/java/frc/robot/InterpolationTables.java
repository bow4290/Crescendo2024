package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.OldSubsystems.NewWrivot;

/**
 * A class made to store all of our interpolation tables, using {@link InterpolatingDoubleTreeMap}.
 */
public class InterpolationTables {

  /**
   * A interpolation table for shooting speaker, using the v1 pivot. Contains an array of key value pairs with working Distance-Angle setups.
   * @apiNote Key: Distance to tag from limelight
   * @apiNote Value: Angle of pivot
   */
  public static InterpolatingDoubleTreeMap speakerDistanceAngle = new InterpolatingDoubleTreeMap();

  // TODO: Find the minimum distance we can shoot from (bump / subwoofer shoot) / max height
  private static double speakerMinimumDistance = 0;
  // TODO: Find the maximum distance we can reasonably shoot from (hopefully pretty far) / minimum height
  private static double speakerMaximumDistance = 100;

  // Put values for things in here
  static {
    // Farthest Shot (Farthest Distance / Shortest Angle)
    speakerDistanceAngle.put(speakerMaximumDistance, 10.0);
    // Bump Shoot (Closest Distance / Highest Angle)
    speakerDistanceAngle.put(speakerMinimumDistance, NewWrivot.BotStates.SPEAKER.getPivotDeg());
  }

  /**
   * Backup function, the get method should already do this. Clips interpolation past the upper or lower limits, as to not get unreasonable values. Is essentially the same as {@link InterpolatingDoubleTreeMap#get(Double)}
   * @param keyValue
   * @return
   */
  public static double interpolateSpeakerClipped(double keyValue){

    if (keyValue < speakerMinimumDistance){
      return speakerDistanceAngle.get(speakerMinimumDistance);
    } else if (keyValue > speakerMaximumDistance){
      return speakerDistanceAngle.get(speakerMaximumDistance);
    }

    return speakerDistanceAngle.get(keyValue);

  }

}
