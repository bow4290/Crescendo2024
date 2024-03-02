package frc.robot;

import java.lang.Math;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class MovingLimelightVision {
  //TO-DO: Find values in inches
  private static final double limelightArmDistance = 1;
  private static final double limelightPivotDistance = 1;
  private static final double pivotFloorDistance = 1;
  //Found
  private static final double bottomOfSpeakerAprilTagDistanceToFloor = 51.875;
  private static final double speakerAprilTagHeight = 10;
  private static final double speakerAprilTagDistanceToFloor = bottomOfSpeakerAprilTagDistanceToFloor+(speakerAprilTagHeight/2);
  //TO-DO: Find values in degrees
  private static final double limelightAngleToArm = 15;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public double getLimelightXAngleFromFloor(double pivotAngle) {
    NetworkTableEntry ty = table.getEntry("ty");
    double verticalTargetOffsetAngle = ty.getDouble(0.0);
    return limelightAngleToArm + pivotAngle + verticalTargetOffsetAngle;
  }

  public double getLimelightXAngleToTag(){
    NetworkTableEntry ty = table.getEntry("ty");
    return ty.getDouble(0.0);
  }
  
  public double getLimelightYAngleToTag(){
    NetworkTableEntry tx = table.getEntry("tx");
    return tx.getDouble(0.0);
  }

  public double getLimelightDistanceToTag(double pivotAngle) {
    double limelightYDistanceToPivot = Math.sin(pivotAngle)*limelightPivotDistance;
    double limeLightFloorDistance = limelightYDistanceToPivot+pivotFloorDistance;

    double angleToGoalDegrees = getLimelightXAngleFromFloor(pivotAngle);
    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
    double limelightDistanceToGoal = (speakerAprilTagDistanceToFloor - limeLightFloorDistance) / Math.tan(angleToGoalRadians);
    return limelightDistanceToGoal;
  }
}
