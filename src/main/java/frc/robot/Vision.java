package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
  public static final double SPEAKER_APRIL_TAG_HEIGHT = 56.875;
  
  private static final double limelightAngle = 32.5;
  //TO-DO: Find values in inches
  private static final double limelightHeightFromFloor = 1;
  
  // April tag height for speaker april tags

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public static double getLimelightDistanceToTag(double aprilTagHeightFromFloor){
    return (aprilTagHeightFromFloor - limelightHeightFromFloor) / Math.tan((getLimelightTY() + limelightAngle) * (3.14159 / 180.0));
  }

  public static double getLimelightTY(){
    NetworkTableEntry ty = table.getEntry("ty");
    return ty.getDouble(0.0);
  }
  
  public static double getLimelightTX(){
    NetworkTableEntry tx = table.getEntry("tx");
    return tx.getDouble(0.0);
  }

  public static boolean getLimelightTV(){
    NetworkTableEntry tv = table.getEntry("tv");
    return tv.getBoolean(false); 
  }

  public static int getLimelightTID(){
    NetworkTableEntry tid = table.getEntry("tid");
    return (int) tid.getInteger(0);
  }

  



  //test
}
