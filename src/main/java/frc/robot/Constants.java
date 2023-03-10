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
    public static final double shoulderL = 0.9906;
    public static final double elbowL = 0.9144;
    public static final double shoulderHeight = 11*0.0254;
    public static final double shoulderWeight = 3;
    public static final double elbowMotorWeight = 1;
    public static final double elbowWeight = 4;
    public static final double elbowperMotorTick = 6.28/4096;
    public static final double shoulderperMotorTick = 6.28/4096;
    public static final int numberOfState = 5;
    public static final double shoulderPowerLimit = 0.6;
    public static double[] targetShoulder = {0, -0.46,-0.42,  0.46, 0.42};
    public static double[] targetElbow =    {0,  1.02, 2.00, -1.02, -2.0};
    public static double[] targetX ={0.000, 0.93, 1.32, -0.93, -1.32};
    public static double[] targetY ={0.076, 0.12, 0.91,  0.12,  0.91};
    public static double[] xRange = {0.2, 0.3, 0.23, 0.3, 0.23};
    public static double[] yRange = {0.2, 0.2, 0.20, 0.2, 0.20};
    public static double[] shoulderRestVoltage = {0, 0, 0, 0, 0}; 
    public static double[] elbowRestVoltage = {0, 0, 0, 0, 0};
  }

  public static final class robotLimit{
    public static final double height = (78-11)*0.0254;
    public static final double widthFromCenter = 66 * 0.0254;
    public static final double robotLength = 40 * 0.0254;
    public static final double shoulderRange = 0.87;
    public static final double elbowRange = (175.0/180.0)*3.1415;
  }

  public static final class MagicArmCnsts{
    public static final int kSlotIdxShldr = 0;
    public static final int kSlotIdxElbow = 0;
    public static final int kPIDLoopIdxShldr = 0;
    public static final int kPIDLoopIdxElbow = 0;
    public static final int kTimeoutMs = 30;
	  /**
	    * Gains used in Motion Magic, to be adjusted accordingly
      * Gains(kp, ki, kd, kf, izone, peak output);
    */
    public static final Gains kGainsShldr = new Gains(8.0, 0.0001, 100.0, 0.1, 0, 1.0);
    public static final Gains kGainsElbow = new Gains(7.0, 0.0001, 70.0, 0.1, 0, 1.0);
  }
}
