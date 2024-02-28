package frc.lib.math;

public class Conversions {
    
    /**
     * @param wheelRPS Wheel Velocity: (in Rotations per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Meters per Second)
     */
    public static double RPSToMPS(double wheelRPS, double circumference){
        double wheelMPS = wheelRPS * circumference;
        return wheelMPS;
    }

    /**
     * @param wheelMPS Wheel Velocity: (in Meters per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Rotations per Second)
     */
    public static double MPSToRPS(double wheelMPS, double circumference){
        double wheelRPS = wheelMPS / circumference;
        return wheelRPS;
    }

    /**
     * @param wheelRotations Wheel Position: (in Rotations)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Distance: (in Meters)
     */
    public static double rotationsToMeters(double wheelRotations, double circumference){
        double wheelMeters = wheelRotations * circumference;
        return wheelMeters;
    }

    /**
     * @param wheelMeters Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Position: (in Rotations)
     */
    public static double metersToRotations(double wheelMeters, double circumference){
        double wheelRotations = wheelMeters / circumference;
        return wheelRotations;
    }

    public static double metersToNauticalMiles(double meters){
        return meters / 1852;
    }

    public static double nauticalMilesToMeters(double nauticalMiles){
        return nauticalMiles * 1852;
    }

    /**
     * Utility function meant ot convert degrees to rotations.
     * @param degrees Input degrees. Value of type double.
     * @return Rotations.
     */
    public static double degToRotations(double degrees){
        return degrees / 360;
    }

    /**
     * A utility function meant to convert degress to rotations, with a gear ratio included in the input.
     * @param degrees Input degrees. Value of type double. 
     * @param gearRatio Input gear ratio, in the form of the final value of it (ex. 4:2 would be 4/2). Value Of type double
     * @return Rotations, with gear ratio math applied.
     */
    public static double degToRotationsGearRatio(double degrees, double gearRatio){
        return (degrees / 360) * gearRatio;
    }

    public static double rpmToRps(double inputRPM){
      return inputRPM / 60;
    }

    public static double rpmToRpsGearRatio(double inputRPM, double gearRatio){
      return rpmToRps(inputRPM) * gearRatio;
    }
}