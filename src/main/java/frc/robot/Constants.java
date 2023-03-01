// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static final class ArmConstants {
    public static final int shoulderCAN_ID = 15;
    public static final int elbowCAN_ID = 14;
    public static final double shoulderArmLength = 0.9906;
    public static final double elbowArmLength = 0.9144;
    public static final double shoulderHeight = 11*0.0254;
    public static final double shoulderWeight = 3;
    public static final double elbowMotorWeight = 1;
    public static final double elbowWeight = 4;
    public static final double elbowperMotorTick = 6.28/4096;
    public static final double shoulderperMotorTick = 6.28/4096;
    public static final int numberOfState = 5;
    public static double[] targetShoulder = {0,0.8,-0.8,0.6,-0.6};
    public static double[] targetElbow = {0,1.7,-1.7,2.9,-2.9};
    public static double[] targetX ={0, 1.43, -1.43, 1.24, -1.24};
    public static double[] targetY ={0.0762,0.12, 0.12, 1.42, 1.42};
    public static double[] xRange = {0.2, 0.2, 0.2, 0.2, 0.2};
    public static double[] yRange = {0.1, 0.1, 0.1, 0.1, 0.1};
    public static double[] shoulderRestVoltage = {0.014, -0.2, 0.2, -0.1, 0.1}; 
    public static double[] elbowRestVoltage = {0, -0.2, 0.2, -0.1, 0.1};
  }

  public static final class robotLimit{
    public static final double height = 78*0.0254;
    public static final double widthFromCenter = 66 * 0.0254;
    public static final double robotLength = 40 * 0.0254;
}
}
