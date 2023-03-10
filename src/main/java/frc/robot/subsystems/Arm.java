// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.robotLimit;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Arm extends SubsystemBase {
  private WPI_TalonSRX shoulderMotor;
	private WPI_TalonSRX elbowMotor;
  private double shoulderAngle = 0;
  private double elbowAngle = 0;

  public Arm() {
    shoulderMotor = new WPI_TalonSRX(ArmConstants.shoulderCAN_ID);
		elbowMotor = new WPI_TalonSRX(ArmConstants.elbowCAN_ID);
		elbowMotor.setInverted(true);
    shoulderMotor.setInverted(true);
    shoulderMotor.setNeutralMode(NeutralMode.Coast);
		elbowMotor.setNeutralMode(NeutralMode.Coast);
  }

  public boolean isXYInLimit(double x, double y){
    if(y < -ArmConstants.shoulderHeight + 0.1) return false;
    if(y + ArmConstants.shoulderHeight > robotLimit.height) return false;
    if(Math.abs(x) > robotLimit.widthFromCenter) return false;
    if(Math.abs(x) < robotLimit.robotLength / 2 && y < 0)return false;
    return true;
  }

  public double[] getXY(){
    return getXY(shoulderAngle, elbowAngle);
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

  public void run(double shoulderPower, double elbowPower){
    elbowMotor.set(elbowPower);
    shoulderMotor.set(MathUtil.clamp(shoulderPower, ArmConstants.shoulderPowerLimit, - ArmConstants.shoulderPowerLimit));
    SmartDashboard.putNumber("shoulderPower", shoulderPower);
    SmartDashboard.putNumber("elbowPower", elbowPower);
  }
      
  public double getElbowAngle(){return elbowAngle;}
    
  public double getShoulderAngle(){return shoulderAngle;}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double elbowTicks = elbowMotor.getSelectedSensorPosition();
    double shoulderTicks = shoulderMotor.getSelectedSensorPosition();
    elbowAngle = elbowTicks * ArmConstants.elbowperMotorTick;
    shoulderAngle = shoulderTicks * ArmConstants.shoulderperMotorTick;
    SmartDashboard.putNumber("ElbowEncoder", elbowTicks);
		SmartDashboard.putNumber("ShoulderEncoder", shoulderTicks);
    SmartDashboard.putNumber("currentShoulderAngle", shoulderAngle);
    SmartDashboard.putNumber("currentElbowAngle", elbowAngle);
  }
}
