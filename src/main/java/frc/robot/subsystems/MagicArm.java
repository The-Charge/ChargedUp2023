// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MagicArmCnsts;
import frc.robot.Constants.robotLimit;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

// In this version, counterclockwise rotation is positive and clockwise is negative.
// The cartesian coordinate is defined as the origin at the shoulder totation axis, negative rotation of shoulder brings it to the 1st quad.
// The units are in the MKS system.
// The team is working towards a version that with a different setup that has been agreed upon on 3/4/2023.
// Restricitions: Shoulder and Elbow are in the same quadrant within some error range.    
public class MagicArm extends SubsystemBase {
  private WPI_TalonSRX shldrMtr;
  private WPI_TalonSRX elbowMtr;
  private double shldrAngl = 0;
  private double elbowAngl = 0;

  public MagicArm() {
    shldrMtr = new WPI_TalonSRX(ArmConstants.shoulderCAN_ID);
    elbowMtr = new WPI_TalonSRX(ArmConstants.elbowCAN_ID);

    /* Factory default hardware to prevent unexpected behavior */
    shldrMtr.configFactoryDefault();
    elbowMtr.configFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    shldrMtr.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, MagicArmCnsts.kPIDLoopIdxShldr,
        MagicArmCnsts.kTimeoutMs);
    elbowMtr.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, MagicArmCnsts.kPIDLoopIdxElbow,
        MagicArmCnsts.kTimeoutMs);

    /*
     * set deadband to super small 0.005 (0.5 %). The default deadband is 0.04 (4 %)
     */
    shldrMtr.configNeutralDeadband(0.005, MagicArmCnsts.kTimeoutMs);
    elbowMtr.configNeutralDeadband(0.005, MagicArmCnsts.kTimeoutMs);

    /* Configure Talon SRX Output and Sensor direction accordingly */
    // shldrMtr.setSensorPhase(false);
    shldrMtr.setInverted(false);
    elbowMtr.setInverted(true);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    shldrMtr.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, MagicArmCnsts.kTimeoutMs);
    shldrMtr.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, MagicArmCnsts.kTimeoutMs);
    elbowMtr.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, MagicArmCnsts.kTimeoutMs);
    elbowMtr.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, MagicArmCnsts.kTimeoutMs);

    /* Set the peak and nominal outputs */
    shldrMtr.configNominalOutputForward(0, MagicArmCnsts.kTimeoutMs);
    shldrMtr.configNominalOutputReverse(0, MagicArmCnsts.kTimeoutMs);
    shldrMtr.configPeakOutputForward(1, MagicArmCnsts.kTimeoutMs);
    shldrMtr.configPeakOutputReverse(-1, MagicArmCnsts.kTimeoutMs);
    elbowMtr.configNominalOutputForward(0, MagicArmCnsts.kTimeoutMs);
    elbowMtr.configNominalOutputReverse(0, MagicArmCnsts.kTimeoutMs);
    elbowMtr.configPeakOutputForward(1, MagicArmCnsts.kTimeoutMs);
    elbowMtr.configPeakOutputReverse(-1, MagicArmCnsts.kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    shldrMtr.selectProfileSlot(MagicArmCnsts.kSlotIdxShldr, MagicArmCnsts.kPIDLoopIdxShldr);
    shldrMtr.config_kF(MagicArmCnsts.kSlotIdxShldr, MagicArmCnsts.kGainsShldr.kF, MagicArmCnsts.kTimeoutMs);
    shldrMtr.config_kP(MagicArmCnsts.kSlotIdxShldr, MagicArmCnsts.kGainsShldr.kP, MagicArmCnsts.kTimeoutMs);
    shldrMtr.config_kI(MagicArmCnsts.kSlotIdxShldr, MagicArmCnsts.kGainsShldr.kI, MagicArmCnsts.kTimeoutMs);
    shldrMtr.config_kD(MagicArmCnsts.kSlotIdxShldr, MagicArmCnsts.kGainsShldr.kD, MagicArmCnsts.kTimeoutMs);
    elbowMtr.selectProfileSlot(MagicArmCnsts.kSlotIdxElbow, MagicArmCnsts.kPIDLoopIdxElbow);
    elbowMtr.config_kF(MagicArmCnsts.kSlotIdxElbow, MagicArmCnsts.kGainsElbow.kF, MagicArmCnsts.kTimeoutMs);
    elbowMtr.config_kP(MagicArmCnsts.kSlotIdxElbow, MagicArmCnsts.kGainsElbow.kP, MagicArmCnsts.kTimeoutMs);
    elbowMtr.config_kI(MagicArmCnsts.kSlotIdxElbow, MagicArmCnsts.kGainsElbow.kI, MagicArmCnsts.kTimeoutMs);
    elbowMtr.config_kD(MagicArmCnsts.kSlotIdxElbow, MagicArmCnsts.kGainsElbow.kD, MagicArmCnsts.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    shldrMtr.configMotionCruiseVelocity(75, MagicArmCnsts.kTimeoutMs);
    shldrMtr.configMotionAcceleration(100, MagicArmCnsts.kTimeoutMs);
    shldrMtr.configMotionSCurveStrength(5, MagicArmCnsts.kTimeoutMs);
    elbowMtr.configMotionCruiseVelocity(100, MagicArmCnsts.kTimeoutMs);
    elbowMtr.configMotionAcceleration(45, MagicArmCnsts.kTimeoutMs);
    elbowMtr.configMotionSCurveStrength(7, MagicArmCnsts.kTimeoutMs);

    // Need the code from Mr. Curry to set relative sensor values from abosulte
    // sensor values
    /* Zero the sensor once on robot boot up */
    int shldrTick = shldrMtr.getSensorCollection().getPulseWidthPosition() % 4096 - 23;
    if (shldrTick > 2048) shldrTick -= 4096;
    else if (shldrTick < -2048) shldrTick += 4096;
    shldrMtr.setSelectedSensorPosition(shldrTick, MagicArmCnsts.kPIDLoopIdxShldr, MagicArmCnsts.kTimeoutMs);
    int elbowTick = elbowMtr.getSensorCollection().getPulseWidthPosition() % 4096 - 1804;
    if (elbowTick > 2048) shldrTick -= 4096;
    else if (elbowTick < -2048) elbowTick += 4096;
    elbowMtr.setSelectedSensorPosition(-elbowTick, MagicArmCnsts.kPIDLoopIdxElbow, MagicArmCnsts.kTimeoutMs);

    shldrMtr.setNeutralMode(NeutralMode.Brake);
    elbowMtr.setNeutralMode(NeutralMode.Brake);
  }
  public void setCoastMode(){
    elbowMtr.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Determine whether the (_x,_y) cooridnate is allowed by the game rule and will
   * not damage the robot.
   * 
   * @param _x
   * @param _y
   * @return true if the cooridnate is allowed, false otherwise.
   */
  public boolean isXYInLimit(double _x, double _y) {
    if (_y < -ArmConstants.shoulderHeight + 0.02)
      return false; // Not into the floor
    if (_y + ArmConstants.shoulderHeight > robotLimit.height)
      return false; // Not above the height limit
    if (Math.abs(_x) > robotLimit.widthFromCenter)
      return false; // Not extend >48 inch from the robot
    if (Math.abs(_x) < robotLimit.robotLength / 2 && _y < 0)
      return false; // Not below the shoulder mount while inside the robot
    return true;
  }

  public double getLigitX(double _x) {
    return MathUtil.clamp(_x, -robotLimit.widthFromCenter, robotLimit.widthFromCenter);
  }

  public double getLigitY(double _x, double _y) {
    if (Math.abs(_x) < robotLimit.robotLength / 2) {
      return MathUtil.clamp(_y, 0,
          ArmConstants.shoulderL - (Math.sqrt(ArmConstants.elbowL * ArmConstants.elbowL - _x * _x)));
    } else
      return MathUtil.clamp(_y, -ArmConstants.shoulderHeight + 0.02, robotLimit.height - ArmConstants.shoulderHeight);
  }

  /**
   * Get the (x,y) coordinate of the current arm tip.
   * 
   * @return a double array, the first element is the x value and the second
   *         element is the y value.
   */
  public double[] getXY() {
    return getXY(shldrAngl, elbowAngl); // shldrAngl and elbowAngl are always updated during periodic
  }

  /**
   * Get the (x,y) coordinate of the arm tip given a shoulder angle and an elbow
   * angle.
   * 
   * @param _shoulderAngle
   * @param _elbowAngle
   * @return a double array, the first element is the x value and the second
   *         element is the y value.
   */
  public double[] getXY(double _shoulderAngle, double _elbowAngle) {
    double[] xy = new double[2];
    double shoulderHorizen = Math.PI / 2 + _shoulderAngle; // find the angle between x axis and the shoulder
    double elbowHorizen = -Math.PI / 2 + _elbowAngle + _shoulderAngle; // find the angle between + axis and the elbow
    xy[0] = Math.cos(shoulderHorizen) * ArmConstants.shoulderL + Math.cos(elbowHorizen) * ArmConstants.elbowL;
    xy[1] = Math.sin(shoulderHorizen) * ArmConstants.shoulderL + Math.sin(elbowHorizen) * ArmConstants.elbowL;
    return xy;
  }

  /**
   * Determine whether a shoulder and elbow angle pair is allowed by the game
   * rules and won't damage the robot
   * and also the shoulder angle and the elbow angle is within the range of the
   * robot design.
   * 
   * @param _shoulderAngle
   * @param _elbowAngle
   * @return true if the cooridnate is allowed, false otherwise.
   */
  public boolean isAngleInLimit(double _shoulderAngle, double _elbowAngle) {
    double[] xy = getXY(_shoulderAngle, _elbowAngle);
    return isXYInLimit(xy[0], xy[1]) && (Math.abs(_shoulderAngle) < robotLimit.shoulderRange) &&
        (Math.abs(_elbowAngle) < robotLimit.elbowRange);
  }

  /**
   * Calculate desired shoulder and elbow angles to put the arm tip at the (_x,_y)
   * coordinate
   * It uses the triangle formed by the shoulder axle, the elbow axle, and (_x,_y)
   * as the three vertices and use the law of cosine to solve for the angles.
   * 
   * @param _x
   * @param _y
   * @return a double array with the first element is the shoulder angle and the
   *         second element is the elbow angle and
   *         the third element is 1 if a solution can be found and -1 for no
   *         feasible solution.
   */
  public double[] getAngles(double _x, double _y) {
    double[] angles = new double[3];
    if (Math.abs(_x) < 0.01) { // avoiding divide by 0 problem
      angles[0] = 0;
      angles[1] = 0;
      angles[2] = 1;
      return angles;
    }
    double thirdSide2 = _x * _x + _y * _y;
    double thirdSide = Math.sqrt(thirdSide2); // Find the length of the segment connecting (_x,_y) point to the shoulder
                                              // axle
    if (thirdSide + ArmConstants.elbowL < ArmConstants.shoulderL ||
        thirdSide > ArmConstants.elbowL + ArmConstants.shoulderL)
      angles[2] = -1; // No triangle exists
    else {
      double shoulder2 = ArmConstants.shoulderL * ArmConstants.shoulderL;
      double elbow2 = ArmConstants.elbowL * ArmConstants.elbowL;
      double oppositeElbowAngle = Math.acos((thirdSide2 + shoulder2 - elbow2) / 2 / ArmConstants.shoulderL / thirdSide); // Law
                                                                                                                         // of
                                                                                                                         // cosine
      angles[1] = Math.acos((shoulder2 + elbow2 - thirdSide2) / 2 / ArmConstants.elbowL / ArmConstants.shoulderL); // 2nd
                                                                                                                   // application
                                                                                                                   // of
                                                                                                                   // Law
                                                                                                                   // of
                                                                                                                   // cosine
      angles[0] = Math.abs(Math.PI / 2 - oppositeElbowAngle - Math.atan(_y / Math.abs(_x))); // convert angle from +x
                                                                                             // axis to the shoulder
                                                                                             // motor angle
      if (_x > 0)
        angles[0] = -angles[0]; // the triangle is solved as in the 1st quadrant, now compensate if it is in the
                                // 2nd quad.
      if (angles[0] > 0)
        angles[1] = -angles[1]; // Forcing the elbow and the shoulder in the same quadrant, also takes care of
                                // the elbow in the 2nd quad math
      if (Math.abs(angles[0]) > robotLimit.shoulderRange || Math.abs(angles[1]) > robotLimit.elbowRange)
        angles[2] = -1; // tell user to ignore solutions because it is outside the range set by the
                        // robot design
      else
        angles[2] = 1; // acceptable solution
    }
    return angles;
  }

  /**
   * Run motion magic to set the shoulder and the elbow angle
   * 
   * @param _shldrAngl
   * @param _elbowAngl
   */
  public void run(double _shldrAngl, double _elbowAngl) {
    shldrMtr.set(ControlMode.MotionMagic, _shldrAngl / ArmConstants.shoulderperMotorTick);
    elbowMtr.set(ControlMode.MotionMagic, _elbowAngl / ArmConstants.elbowperMotorTick);
  }

  public void runElbow(double _elbowAngl){
    elbowMtr.set(ControlMode.MotionMagic, _elbowAngl / ArmConstants.elbowperMotorTick);
  }

  public double getElbowAngle() {
    return elbowAngl;
  }

  public double getShoulderAngle() {
    return shldrAngl;
  }

  @Override
  public void periodic() {
    double elbowTicks = elbowMtr.getSelectedSensorPosition();
    double shldrTicks = shldrMtr.getSelectedSensorPosition();
    elbowAngl = elbowTicks * ArmConstants.elbowperMotorTick;
    shldrAngl = shldrTicks * ArmConstants.shoulderperMotorTick;
    SmartDashboard.putNumber("ElbowEncoder", elbowTicks);
    SmartDashboard.putNumber("ShoulderEncoder", shldrTicks);
    SmartDashboard.putNumber("currentShoulderAngle", shldrAngl / 3.14 * 180.0);
    SmartDashboard.putNumber("currentElbowAngle", elbowAngl / 3.14 * 180.0);
  }

  public boolean isArmTipInsideRobotX() {
    double[] xy = getXY();
    return Math.abs(xy[0]+0.1) < robotLimit.robotLength / 2;
  }

  /**
   * Move the arm tip to the desired (x,y) coordinates.
   * The starting position needs to be close to a neutral position: shoulder up,
   * elbow down.
   * 
   * @param _x
   * @param _y
   * @return true if a solution is available, otherwise false.
   */
  public boolean moveTowardXYFromNeutral(double _x, double _y) {
    if (isXYInLimit(_x, _y)) {
      double[] angles = getAngles(_x, _y);
      if (angles[2] > 0) {
        if (Math.abs(angles[1] - elbowAngl) < 0.05)
          run(angles[0], angles[1]); // elbow already at target, move shoulder
        else {
          double[] xy = getXY();
          if (Math.abs(xy[0]) < robotLimit.robotLength / 2)
            run(shldrAngl, angles[1]); // only move the elbow if the arm tip has not cleared the robot,
          else if (xy[1] < -ArmConstants.shoulderHeight + 0.02)
            run(shldrAngl * 0.99, angles[1]); // move the elbow and raise the shoulder a little if the arm tip is at the
                                              // ground
          else
            run(angles[0], angles[1]); // move both arms if the arm tip has cleared the robot and not on the ground.
        }
        return true;
      } else
        return false;
    } else
      return false;
  }

  /**
   * To lower the arm tip to the neutral position: shoulder up, elbow down
   */
  public void moveTowardNeutral(){
    if(Math.abs(elbowAngl) > 2.3)run(shldrAngl, 0); //if the arm tip is above the limit, lower the elbow only
    else{
      if (Math.abs(shldrAngl) < ArmConstants.shoulderAngleToSafeSwingElbowThrough)run(0,0); //if the shoulder is at 0, lower the elbow to 0
      else run(0, elbowAngl); //if the shoulder is not at 0, move only the shoulder to 0
    }
  }

  /**
   * Move the arm tip to the desired (x,y) coordinates.
   * use moveTOXYFromNeutral if the starting position is close to neutral since
   * that method has less calculations.
   * 
   * @param _x
   * @param _y
   * @return true if a solution is available, otherwise false.
   */
  public boolean moveTowardXY(double _x, double _y) {
    if (isXYInLimit(_x, _y)) {
      double[] angles = getAngles(_x, _y);
      if (angles[2] > 0) { // a solution is found
        double[] xy = getXY();
        if (xy[0] * _x > 0)
          moveTowardXYFromNeutral(_x, _y); // the current and the desired arm tip positions at the same side of the
                                           // robot
        else {
          if (Math.abs(shldrAngl) < ArmConstants.shoulderAngleToSafeSwingElbowThrough)
            run(0, angles[1]); // optimization to avoid the elbow slow down at the neutral position
          else
            moveTowardNeutral(); // move to neutral position if optimization cannot be safely performed.
        }
        return true;
      } else
        return false;
    }
    return false;
  }
}
