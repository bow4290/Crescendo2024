package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
  //TO-DO: Find values in inches
  private static final double limelightHeightFromFloor = 1;
  //Found
  private static final double aprilTagHeightFromFloor = 56.875;
  //TO-DO: Find values in degrees
  private static final double limelightAngle = 1;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public double getLimelightXAngleToTag(){
    NetworkTableEntry ty = table.getEntry("ty");
    return ty.getDouble(0.0);
  }
  
  public double getLimelightYAngleToTag(){
    NetworkTableEntry tx = table.getEntry("tx");
    return tx.getDouble(0.0);
  }

  public double getLimelightDistanceToTag(){
    return (aprilTagHeightFromFloor - limelightHeightFromFloor) / Math.tan((getLimelightXAngleToTag() + limelightAngle) * (3.14159 / 180.0));
  }


  



  //test
}
