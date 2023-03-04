// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MagicArmCnsts;
import frc.robot.Constants.robotLimit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;


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

    /* set deadband to super small 0.005 (0.5 %). The default deadband is 0.04 (4 %) */
    shldrMtr.configNeutralDeadband(0.005, MagicArmCnsts.kTimeoutMs);
    elbowMtr.configNeutralDeadband(0.005, MagicArmCnsts.kTimeoutMs);

    /* Configure Talon SRX Output and Sensor direction accordingly */
		//shldrMtr.setSensorPhase(false);
		shldrMtr.setInverted(true);
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
    elbowMtr.configMotionCruiseVelocity(80, MagicArmCnsts.kTimeoutMs);
		elbowMtr.configMotionAcceleration(45, MagicArmCnsts.kTimeoutMs);
    elbowMtr.configMotionSCurveStrength(7, MagicArmCnsts.kTimeoutMs);


    /* Zero the sensor once on robot boot up */
    shldrMtr.setSelectedSensorPosition(0, MagicArmCnsts.kPIDLoopIdxShldr, MagicArmCnsts.kTimeoutMs);
    elbowMtr.setSelectedSensorPosition(0, MagicArmCnsts.kPIDLoopIdxElbow, MagicArmCnsts.kTimeoutMs);

    shldrMtr.setNeutralMode(NeutralMode.Brake);
		elbowMtr.setNeutralMode(NeutralMode.Coast);
  }

  public boolean isXYInLimit(double x, double y){
    if(y < -ArmConstants.shoulderHeight + 0.1) return false;
    if(y + ArmConstants.shoulderHeight > robotLimit.height) return false;
    if(Math.abs(x) > robotLimit.widthFromCenter) return false;
    if(Math.abs(x) < robotLimit.robotLength / 2 && y < 0)return false;
    return true;
  }

  public double[] getXY (double shoulderParameter, double elbowParameter){
    double[] xy = new double[2];
    double shoulderHorizen = Math.PI/2 + shoulderParameter;
    double elbowHorizen = -Math.PI/2 + elbowParameter + shoulderParameter;
    xy[0] = Math.cos(shoulderHorizen) * ArmConstants.shoulderL + Math.cos(elbowHorizen) * ArmConstants.elbowL;
    xy[1] = Math.sin(shoulderHorizen) * ArmConstants.shoulderL + Math.sin(elbowHorizen) * ArmConstants.elbowL;
    return xy; 
  }

  public boolean isAngleInLimit(double targetShoulderAngle, double targetElbowAngle){
    double[] xy = getXY(targetShoulderAngle, targetElbowAngle);
    return isXYInLimit(xy[0],xy[1]);
  }

  public double[] getAngles(double targetX, double targetY){
    // first value is for shoulder angle to achieve the target x,y
    // second value is for elbow angle to achieve the target x,y
    // third value is 1 if yes and -1 for no feasible solution.
    double[] angles = new double[3];
    if(Math.abs(targetX) < 0.01){
      angles[0] = 0;
      angles[1] = 0;
      angles[2] = 1;
      return angles;
    }
    double thirdSide2 = targetX*targetX + targetY*targetY;
    double thirdSide = Math.sqrt(thirdSide2);
    if (thirdSide + ArmConstants.elbowL < ArmConstants.shoulderL  || 
      thirdSide > ArmConstants.elbowL + ArmConstants.shoulderL) angles[2] = -1;
    else{
      angles[2] = 1;
      double shoulder2 = ArmConstants.shoulderL*ArmConstants.shoulderL;
      double elbow2 = ArmConstants.elbowL*ArmConstants.elbowL;
      double oppositeElbowAngle = Math.acos((thirdSide2 + shoulder2 - elbow2) / 2 / ArmConstants.shoulderL / thirdSide);
      angles[1] = Math.acos((shoulder2 + elbow2 - thirdSide2) / 2 / ArmConstants.elbowL / ArmConstants.shoulderL);
      angles[0] = Math.abs(Math.PI/2 - oppositeElbowAngle - Math.atan(targetY/Math.abs(targetX)));
      if (targetX > 0) angles[0] = -angles[0];
      if (angles[0] > 0)angles[1] = -angles[1];
    }   
    return angles;
  }

  public void run(double _shldrAngl, double _elbowAngl){ 
    shldrMtr.set(ControlMode.MotionMagic, _shldrAngl/ArmConstants.shoulderperMotorTick);
    elbowMtr.set(ControlMode.MotionMagic, _elbowAngl/ArmConstants.elbowperMotorTick);
  }
      
  public double getElbowAngle(){return elbowAngl;}
    
  public double getShoulderAngle(){return shldrAngl;}

  @Override
  public void periodic() {
    double elbowTicks = elbowMtr.getSelectedSensorPosition();
    double shldrTicks = shldrMtr.getSelectedSensorPosition();
    elbowAngl = elbowTicks * ArmConstants.elbowperMotorTick;
    shldrAngl = shldrTicks * ArmConstants.shoulderperMotorTick;
    SmartDashboard.putNumber("ElbowEncoder",    elbowTicks);
		SmartDashboard.putNumber("ShoulderEncoder", shldrTicks);
    SmartDashboard.putNumber("currentShoulderAngle", shldrAngl);
    SmartDashboard.putNumber("currentElbowAngle",    elbowAngl);
  }
}
