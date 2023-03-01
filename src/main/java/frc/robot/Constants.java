// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
public class Constants {

   public static final class DriveConstants {
      public static final int kLeftMotor1Port = 16; // LeftFront
      public static final int kLeftMotor2Port = 17; // LeftRear
      public static final int kRightMotor1Port = 3; // RightFront
      public static final int kRightMotor2Port = 2; // RightRear
      public static final int shifterChannel = 1; // Shifter
      public static final int clawChannel = 2; // Claw
   }
   
   public static final class VisionConstants {
      public static final String cameraName = "IMX219";

      public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(9.25);
      public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(19);
   
      public static final double nodeSideDistanceMeters = Units.inchesToMeters(6.25 + 0);
   
      public static final double fieldLength = Units.inchesToMeters(651.25);
      public static final double fieldWidth = Units.inchesToMeters(315.5);
   
      public static final Transform3d robotToCamera = new Transform3d(new Translation3d(0.31, 0.07, Units.inchesToMeters(9.5)), new Rotation3d(0, 19, 0));
      public static final Transform3d cameraToRobot = robotToCamera.inverse();
   
      //31,7
      public static final double LINEAR_P = 1;
      public static final double LINEAR_D = 0;
      public static final double ANGULAR_P = 0.05; //0.05
      public static final double ANGULAR_D = 0;
   }
}
