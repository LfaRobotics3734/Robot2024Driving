// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;

  }

  public static class IO{
    //CAN ID constants swerve drive
    public static int frontLeftDrive = 3;
    public static int frontLeftTurn = 4;

    public static int frontRightDrive = 1;
    public static int frontRightTurn = 2;
    
    public static int backLeftDrive = 7;
    public static int backLeftTurn = 8;

    public static int backRightDrive = 5;
    public static int backRightTurn = 6;

    public static int driveController = 0;
    public static int armController = 1;

    public static double kDriveDeadband = 0.05;
  }

  public static class Drive{
      public static double positionConversionFactor = PhysicalParams.wheelDiameters * Math.PI / PhysicalParams.drivingMotorReduction;
      public static double velocityConversionFactor = positionConversionFactor / 60.0;
      
      public static double p = 0.1;
      public static double i = 0;
      public static double d = 0;
      public static double ff = 1/PhysicalParams.drivingFreeSpeedRPS;

      //ratio for gears
      public static double lowGear = 0.225;
      public static double highGear = 1.0;
  }
  

  public static class Turn{

      public static double positionConversionFactor = 2 * Math.PI;
      public static double velocityConversionFactor = 2 * Math.PI/60;

      public static double p = 1;
      public static double i = 0;
      public static double d = 0;
      public static double ff = 0;

      public static double[] angleOffsets = {0, -Math.PI/2, Math.PI/2, Math.PI};
      //frontRight, frontLeft, backRight, backLeft

  }

  public static class PhysicalParams{

      //in meters
      public static double freeSpeedRPM = 5676;
      public static double freeSpeedRPS = freeSpeedRPM/60;
      public static double wheelDiameters = 0.0762;
      public static double wheelCircumference = wheelDiameters * Math.PI;
      public static double drivingMotorPinionTeeth = 13;
      public static double bevelGear = 45;
      public static double firstStageSpurGear = 22;
      public static double bevelPinion = 15;
      public static double drivingMotorReduction = (bevelGear * firstStageSpurGear)/(drivingMotorPinionTeeth*bevelPinion);
      public static double drivingFreeSpeedRPS = (freeSpeedRPS * wheelCircumference)/drivingMotorReduction;

  }

  public static class DriveConstants{
    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%) (was 1.3)
    public static final double kRotationalSlewRate = 3; // percent per second (A = 100%)

    public static final double maxSpeed = 4;
    public static final double maxRotation = Math.PI;

    public static final double baseDimensions = Units.inchesToMeters(23);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(baseDimensions / 2, baseDimensions / 2),
        new Translation2d(baseDimensions / 2, -baseDimensions / 2),
        new Translation2d(-baseDimensions / 2, baseDimensions / 2),
        new Translation2d(-baseDimensions / 2, -baseDimensions / 2));
    
    public static final Pose2d initialPose = new Pose2d(0, 0, new Rotation2d(0));
  }

  public static class Autonomous{

    public static final double FIELD_HEIGHT = 8.0137;
    public static final int autoPath = 3;

    // Balance Constants
    public static final double balanceSpeed = 0.07; // In m/s
    public static final double balanceOnAngle = 2.5; // In degrees
    public static final double balanceOffAngle = 2;
    public static final int loops = 50;

    public static double TRANSLATION_PID=  11.4;
    public static double ROTATION_PID= 5;
  }

  public static class ArmConstants{
    public static final double SHOULDER_BASE_ANGLE =  -72.66898321241575;
    public static final double ELBOW_BASE_ANGLE =  148;

    public static final double SHOULDER_FLOOR_ANGLE = -66.46035484048865;
    public static final double ELBOW_FLOOR_ANGLE =63.43951432392655;
    


    
    public static final double SHOULDER_HIGH_POLE_ANGLE=  19.227076946447355;
    public static final double ELBOW_HIGH_POLE_ANGLE = 10.212097260941283;

    public static final double SHOULDER_QUICK_CUBE_ANGLE = -74.64;
    public static final double ELBOW_QUICK_CUBE_ANGLE = 116.78;
    


    public static final double SHOULDER_LOW_POLE_ANGLE=-19.7555241849255;
    public static final double ELBOW_LOW_POLE_ANGLE= 54.41065032215853;
    
    public static final double SHOULDER_LOW_PLATFORM_ANGLE=-20.075481974412067;
    public static final double ELBOW_LOW_PLATFORM_ANGLE = 41.96732681183557;

    public static final double SHOULDER_HIGH_PLATFORM_ANGLE=6.5;// new values 38.11895073113292
    public static final double ELBOW_HIGH_PLATFORM_ANGLE=16.026402875404735; // new values -34.086191463277224
  
    public static final double NEW_SHOULDER_HIGH_PLATFORM_ANGLE=38.11895073113292;
    public static final double NEW_ELBOW_HIGH_PLATFORM_ANGLE=-34.086191463277224;
    

    public static  double SHOULDER_SUBSTATION_ANGLE=  31.082704839693722;
    public static  double ELBOW_SUBSTATION_ANGLE=  -23.20844959485292;


    public static final double SHOULDER_LENGTH = 24.12533981;
    public static final double ELBOW_LENGTH = 38.237;
    public static final double SHOULDER_HEIGHT = 30.375;

    public static final double LIMELIGHT_SHOULDER_ANGLE= -73.5096072562566;
    public static final double LIMELIGHT_ELBOW_ANGLE= 99.06298490452583;

    public static final double ARM_MAX_SPEED = 0.5;
  }
}
