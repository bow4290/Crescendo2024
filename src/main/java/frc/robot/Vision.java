package frc.robot;

import java.lang.Math;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
  //TO-DO: Find values in inches
  private static final double limelightArmDistance = 1;
  private static final double limelightPivotDistance = 1;
  private static final double pivotFloorDistance = 1;
  //Found
  private static final double bottomOfSpeakerAprilTagDistanceToFloor = 51.875;
  private static final double speakerAprilTagHeight = 10;
  private static final double speakerAprilTagDistanceToFloor = bottomOfSpeakerAprilTagDistanceToFloor+(speakerAprilTagHeight/2);
  //This is based off previously made variables
  private static final double limelightAngleToArm = Math.atan(limelightArmDistance/limelightPivotDistance);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public double getLimelightAngleToTag(double pivotAngle) {
    return limelightAngleToArm + pivotAngle;
  }

  public double getLimelightDistanceToTag(double pivotAngle) {
    NetworkTableEntry ty = table.getEntry("ty");
    double verticalTargetOffsetAngle = ty.getDouble(0.0);

    double limelightYDistanceToPivot = Math.sin(pivotAngle)*limelightPivotDistance;
    double limeLightFloorDistance = limelightYDistanceToPivot+pivotFloorDistance;

    double angleToGoalDegrees = getLimelightAngleToTag(pivotAngle);
    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
    double limelightDistanceToGoal = (speakerAprilTagDistanceToFloor - limeLightFloorDistance) / Math.tan(angleToGoalRadians);
    return limelightDistanceToGoal;
  }
}
