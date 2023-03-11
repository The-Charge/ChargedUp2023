// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
public class Constants {

   public static final class DriveConstants {
      public static final int kLeftMotorFrontPort = 16; // LeftFront
      public static final int kLeftMotorRearPort = 17; // LeftRear
      public static final int kRightMotorFrontPort = 2; // RightFront
      public static final int kRightMotorRearPort = 3; // RightRear
      public static final int shifterChannel = 1; // Shifter
      public static final int clawChannel = 2; // Claw
   }

   public static final class SysIDConstants {
      public static final double ticksPerMeter = 17483.3; // Average from three tests from both encoders

      public static final double leftEncoderTicksPerMeter = 18850; //17362, 92% is 
      public static final double rightEncoderTicksPerMeter = 18850;

      //volt constraints
      public static final double ksVolts = 2; 
      public static final double kvVoltSecondsPerMeter = 2.5; 
      public static final double kaVoltSecondsSquaredPerMeter = .25; 

      //PID values
      public static final double kPDriveVel = 9; 
      public static final double kIDriveVel = 3; 
      public static final double kDDriveVel = .1; 

      public static final double kTrackwidthMeters = 0.60008;
      public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);

      //max speed and acceleration 
      public static final double kMaxSpeedMetersPerSecond = 3; 
      public static final double kMaxAccelerationMetersPerSecondSquared = 2; 

      public static final double kRamseteB = 2;
      public static final double kRamseteZeta = 0.7;

      public static final double NUMBER_OF_PATHWAYS = 14; //list of pathways #ed
   }

}
