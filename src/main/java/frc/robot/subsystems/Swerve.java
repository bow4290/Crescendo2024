package frc.robot.subsystems;

import frc.lib.swerve.SwerveConstants;
import frc.lib.swerve.SwerveModule;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Swerve() {
        gyro = new Pigeon2(SwerveConstants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.Swerve.Mod0.constants),
            new SwerveModule(1, SwerveConstants.Swerve.Mod1.constants),
            new SwerveModule(2, SwerveConstants.Swerve.Mod2.constants),
            new SwerveModule(3, SwerveConstants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(SwerveConstants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        // Autobuilder used by PathPlannerLib
        AutoBuilder.configureHolonomic(
        this::getPose, 
        this::setPose, 
        () -> SwerveConstants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates()), 
        this::driveRobotRelative,
        new HolonomicPathFollowerConfig(
          new PIDConstants(SwerveConstants.Swerve.driveKP, SwerveConstants.Swerve.angleKI, SwerveConstants.Swerve.driveKD),
          new PIDConstants(SwerveConstants.Swerve.angleKP, SwerveConstants.Swerve.angleKI, SwerveConstants.Swerve.driveKD),
          SwerveConstants.Swerve.maxSpeed,
          0.45,
          new ReplanningConfig()), 
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()){
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, 
        this);

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            SwerveConstants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    // For pathplanner
    public void driveRobotRelative(ChassisSpeeds givenChassisSpeeds){
      SwerveModuleState[] givenSwerveModStates = SwerveConstants.Swerve.swerveKinematics.toSwerveModuleStates(givenChassisSpeeds);

      for(SwerveModule mod : mSwerveMods){
        mod.setDesiredState(givenSwerveModStates[mod.moduleNumber], false);
      }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber(String.format("Swerve Mod: %d CANcoder", mod.moduleNumber), mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber(String.format("Swerve Mod: %d Angle", mod.moduleNumber), mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber(String.format("Swerve Mod: %d Velocity", mod.moduleNumber), mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("Gyro", gyro.getYaw().getValueAsDouble());
    }
}