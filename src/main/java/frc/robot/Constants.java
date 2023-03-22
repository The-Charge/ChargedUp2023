// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 16;
    public static final int kLeftMotor2Port = 17;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 3;
    public static final int shifterChannel = 1;
    public static final int clawChannel = 2;
  }

  public static final class AutoConstants {
    public static final double climbPowerLimit = 0.57; // was 0.55;
    public static final double climbPowerForwardBias = 0.3;
    public static final double climbPowerBackwardBias = -0.3;
    public static final double climbPitchGain = 0.018;
    public static final double climbPitchDerivativeGain = 0.1; // was 0.6;
    public static final double fastClimbTicks = 25000;
    public static final double headingGain = 0.02;
    public static final double fallPitchPerCycle = 1;
  }
  public static final double InchToMeter = 0.0254;
  public static final double degreeToRadian = Math.PI / 180.0;

  public static final class ArmConstants {
    // All constants are in inches
    // Angles are in degrees
    public static final int shoulderCAN_ID = 1;
    public static final int elbowCAN_ID = 14;
    public static final double shoulderL = 39.0 * InchToMeter;
    public static final double elbowL = 37.0 * InchToMeter;
    public static final double shoulderAngleToSafeSwingElbowThrough = 22.6 * degreeToRadian;
    public static final double shoulderHeight = 11 * InchToMeter;
    public static final double elbowperMotorTick = Math.PI * 2 / 4096;
    public static final double shoulderperMotorTick = Math.PI * 2 / 4096;
    public static final double shoulderScoreDegree = 55.0;
    public static final double hiGoalX = 59.6 * InchToMeter;
    public static final double hiGoalY = 47.0 * InchToMeter;
    public static final double midGoalX = 45.3 * InchToMeter;
    public static final double midGoalY = 33.5 * InchToMeter;
    public static final double stationX = 29.2 * InchToMeter;
    public static final double stationY = 34.2 * InchToMeter;
    public static final double pickUpX = 35.4 * InchToMeter;
    public static final double pickUpY = -8.7 * InchToMeter;
  }

  public static final class robotLimit {
    public static final double height = (78 - 2) * 0.0254;
    public static final double widthFromCenter = 66 * 0.0254;
    public static final double robotLength = 44 * 0.0254;
    public static final double shoulderRange = Units.degreesToRadians(60);
    public static final double elbowRange = (175.0 / 180.0) * 3.1415;
  }

  public static final class MagicArmCnsts {
    public static final int kSlotIdxShldr = 0;
    public static final int kSlotIdxElbow = 0;
    public static final int kPIDLoopIdxShldr = 0;
    public static final int kPIDLoopIdxElbow = 0;
    public static final int kTimeoutMs = 30;
    /**
     * Gains used in Motion Magic, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    public static final Gains kGainsShldr = new Gains(5.0, 0.0001, 100.0, 0.1, 0, 1.0);
    public static final Gains kGainsElbow = new Gains(7.0, 0.0001, 70.0, 0.1, 0, 1.0);
  }

  public static class Gains {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;
    public final int kIzone;
    public final double kPeakOutput;

    public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput) {
      kP = _kP;
      kI = _kI;
      kD = _kD;
      kF = _kF;
      kIzone = _kIzone;
      kPeakOutput = _kPeakOutput;
    }
  }

  public static final class VisionConstants {
    public static final String cameraName = "IMX219";

    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(4.6);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(16.7);

    public static final double nodeSideDistanceMeters = Units.inchesToMeters(6.25 + 0);

    public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double fieldWidth = Units.inchesToMeters(315.5);

    public static final Transform3d robotToCamera = new Transform3d(
        new Translation3d(0.31, 0.07, Units.inchesToMeters(4.6)),
        new Rotation3d(0, 16.7, 0));

    public static final Transform3d cameraToRobot = robotToCamera.inverse();

    // 31,7
    public static final double LINEAR_P = 1;
    public static final double LINEAR_D = 0;
    public static final double ANGULAR_P = 0.05;
    public static final double ANGULAR_D = 0;
  }

  public static final class SysIDConstants {
    // Average from three tests from both encoders
    public static final double ticksPerMeter = 17483.3;

    public static final double leftEncoderTicksPerMeter = 18850;
    public static final double rightEncoderTicksPerMeter = 18850;

    // volt constraints
    public static final double ksVolts = 2;
    public static final double kvVoltSecondsPerMeter = 2.5;
    public static final double kaVoltSecondsSquaredPerMeter = .25;

    // PID values
    public static final double kPDriveVel = 9;
    public static final double kIDriveVel = 3;
    public static final double kDDriveVel = .1;

    public static final double kTrackwidthMeters = 0.60008;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    // max speed and acceleration
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // List of pathways #ed
    public static final int NUMBER_OF_PATHWAYS = 9;

    // Maximum for Velocity ControlMode (Initial Test Starting Pt)
    public static final double MAX_VELOCITY = 7000;
  }
}
