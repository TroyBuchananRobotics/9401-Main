// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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
    public static final int kOperatorControllerPort = 1;
  }
  public static final class CurrentLimit {
    public static final int kTranslation = 50;
    public static final int kRotation = 20;
    public static final int kCubeExt = 20;
    public static final int kConeExt = 20;
    public static final int kCube = 20;
    public static final int kCone = 20;
    public static final int kClaw = 35;
    public static final int kElevator = 40;
    public static final int kWrist = 20;
    public static final int kArm = 20;
  }

  public static final class ModuleConstants {
    private static final double kTranslationGearRatio = 48.0/24.0*30.0/12.0; // Overall gear ratio of the swerve module
    public static final double kRotationGearRatio = 40.0;
    private static final double kWheelDiameter = 2.78/39.37; // Wheel Diameter in meters, may need to be
                                                                         // experimentally determined due to compliance
                                                                         // of floor/tread material

    public static final double kVelocityFactor = (1.0 / kTranslationGearRatio / 60.0) * kWheelDiameter * Math.PI; 
    public static final double kNEOSteerP = 3.0;
    public static final double[] kTurnPID = { 0.800, 0, 0 }; // Defines the PID values for rotation of the serve
                                                             // modules, should show some minor oscillation when no
                                                             // weight is loaded on the modules
  }

  public static final class DriveConstants {
    public static final boolean kUseNEO = false;

    public static final int kFrontLeftDriveMotorPort = 1; // CANID of the Translation SparkMAX
    public static final int kFrontRightDriveMotorPort = 3; // CANID of the Translation SparkMAX
    public static final int kBackLeftDriveMotorPort = 5; // CANID of the Translation SparkMAX
    public static final int kBackRightDriveMotorPort = 7; // CANID of the Translation SparkMAX

    public static final int kFrontLeftTurningMotorPort = 2; // CANID of the Rotation SparkMAX
    public static final int kFrontRightTurningMotorPort = 4; // CANID of the Rotation SparkMAX
    public static final int kBackLeftTurningMotorPort = 6; // CANID of the Rotation SparkMAX
    public static final int kBackRightTurningMotorPort = 8; // CANID of the Rotation SparkMAX

    public static final int kFrontLeftTurningEncoderPort = 0; // Analog Port of the Module Absolute Encoder
    public static final int kFrontRightTurningEncoderPort = 1; // Analog Port of the Module Absolute Encoder
    public static final int kBackLeftTurningEncoderPort = 2; // Analog Port of the Module Absolute Encoder
    public static final int kBackRightTurningEncoderPort = 3; // Analog Port of the Module Absolute Encoder

    public static final double kFrontLeftOffset = 2.917;// Encoder Offset in Radians
    public static final double kFrontRightOffset = 3.035; // Encoder Offset in Radians
    public static final double kBackLeftOffset = 4.820; // Encoder Offset in Radians
    public static final double kBackRightOffset = 0.6675-Math.PI; // Encoder Offset in Radians

    public static final double[] kFrontLeftTuningVals = { 0.015, 0.23, 0.15, 0 }; // {Static Gain, FeedForward,
                                                                                     // Proportional Gain, ModuleID for
                                                                                     // Tuning}
    public static final double[] kFrontRightTuningVals = { 0.015, 0.23, 0.15, 1 }; // {Static Gain, FeedForward,
                                                                                      // Proportional Gain, ModuleID for
                                                                                      // Tuning}
    public static final double[] kBackLeftTuningVals = { 0.015, 0.23, 0.15, 2 }; // {Static Gain, FeedForward,
                                                                                    // Proportional Gain, ModuleID for
                                                                                    // Tuning}
    public static final double[] kBackRightTuningVals = { 0.015, 0.23, 0.15, 3 }; // {Static Gain, FeedForward,
                                                                                     // Proportional Gain, ModuleID for
                                                                                     // Tuning}

    // NOTE: 2910 Swerve the wheels are not directly under the center of rotation
    // (Take into consideration when measuring)
    public static final double kWheelBaseWidth = 16.75/39.37; // Center distance in meters between right and left wheels on
                                                         // robot
    public static final double kWheelBaseLength = 24.75/39.37; // Center distance in meters between front and back wheels on
                                                          // robot

    // Because the swerve modules poisition does not change, define a constant
    // SwerveDriveKinematics for use throughout the code
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBaseLength / 2, kWheelBaseWidth / 2),
        new Translation2d(kWheelBaseLength / 2, -kWheelBaseWidth / 2),
        new Translation2d(-kWheelBaseLength / 2, kWheelBaseWidth / 2),
        new Translation2d(-kWheelBaseLength / 2, -kWheelBaseWidth / 2));

    public static final double kMaxAcceleration = 3.0;
    public static final double kMaxSpeedMetersPerSecond = 4.0; // Maximum Sustainable Drivetrain Speed under Normal
                                                                // Conditions & Battery, Robot will not exceed this
                                                                // speed in closed loop control
    public static final double kTestMaxAcceleration = 1.0;
    public static final double kTestMaxSpeedMetersPerSecond = 1.0;
     
    public static final double kMaxAngularSpeed = 1.5*Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kMaxAngularAccel = 1.5*Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly

    public static final double kInnerDeadband = 0.12; // This value should exceed the maximum value the analog stick may
                                                      // read when not in use (Eliminates "Stick Drift")
    public static final double kOuterDeadband = 0.98; // This value should be lower than the analog stick X or Y reading
                                                      // when aimed at a 45deg angle (Such that X and Y are are
                                                      // maximized simultaneously)

    public static final double[] kKeepAnglePID = { 1.00, 0, 0 }; // Defines the PID values for the keep angle PID

    private static final SwerveModuleState[] kLockedWheelsHelper = kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 1.0));
    public static final SwerveModuleState[] kLockedWheels = {
      new SwerveModuleState(0.0, kLockedWheelsHelper[0].angle.rotateBy(new Rotation2d(Math.PI/2))),
      new SwerveModuleState(0.0, kLockedWheelsHelper[1].angle.rotateBy(new Rotation2d(Math.PI/2))),
      new SwerveModuleState(0.0, kLockedWheelsHelper[2].angle.rotateBy(new Rotation2d(Math.PI/2))),
      new SwerveModuleState(0.0, kLockedWheelsHelper[3].angle.rotateBy(new Rotation2d(Math.PI/2)))
    };


    // Minimum allowable rotation command (in radians/s) assuming user input is
    // squared using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinRotationCommand = DriveConstants.kMaxAngularSpeed
        * Math.pow(DriveConstants.kInnerDeadband, 2);
    // Minimum allowable tranlsation command (in m/s) assuming user input is squared
    // using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinTranslationCommand = DriveConstants.kMaxSpeedMetersPerSecond
        * Math.pow(DriveConstants.kInnerDeadband, 2);

    
  }

  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6; // Sets a voltage compensation value ideally 12.6V
    public static final int PCHID = 20;
    public static final int PDHID = 24;
    public static final double kLoopTime = 0.020;
  }

}
