// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.robotLimit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private WPI_TalonSRX shoulderMotor;
	private WPI_TalonSRX elbowMotor;
  private double shoulderAngle = 0;
  private double elbowAngle = 0;

  public Arm() {
    shoulderMotor = new WPI_TalonSRX(ArmConstants.shoulderCAN_ID);
		elbowMotor = new WPI_TalonSRX(ArmConstants.elbowCAN_ID);
		elbowMotor.setInverted(true);
    shoulderMotor.setNeutralMode(NeutralMode.Coast);
		elbowMotor.setNeutralMode(NeutralMode.Coast);
  }

  public boolean isXYInLimit(double x, double y){
    if(y < -ArmConstants.shoulderHeight) return false;
    if(y + ArmConstants.shoulderHeight > robotLimit.height) return false;
    if(Math.abs(x) > robotLimit.widthFromCenter) return false;
    if(Math.abs(x) < robotLimit.robotLength / 2 && y < 0)return false;
    return true;
  }

  public double[] getXY (double shoulderParameter, double elbowParameter){
    double[] xy = new double[2];
    double shoulderHorizen = Math.PI/2 + shoulderParameter;
    double elbowHorizen = -Math.PI/2 + elbowParameter + shoulderParameter;
    xy[0] = Math.cos(shoulderHorizen) * ArmConstants.shoulderArmLength +
              Math.cos(elbowHorizen) * ArmConstants.elbowArmLength;
    xy[1] = Math.sin(shoulderHorizen) * ArmConstants.shoulderArmLength +
              Math.sin(elbowHorizen) * ArmConstants.elbowArmLength;
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
    double thirdSide = Math.sqrt(targetX * targetX + targetY * targetY);
    if (thirdSide + ArmConstants.elbowArmLength < ArmConstants.shoulderArmLength  || 
    thirdSide > ArmConstants.elbowArmLength + ArmConstants.shoulderArmLength) angles[2] = -1;
    else{
      angles[2] = 1;
      double oppositeElbowAngle = Math.acos((thirdSide * thirdSide + 
        ArmConstants.shoulderArmLength * ArmConstants.shoulderArmLength - 
        ArmConstants.elbowArmLength * ArmConstants.elbowArmLength) / 2 / ArmConstants.shoulderArmLength / thirdSide);
      angles[1] = Math.asin(thirdSide * Math.sin(oppositeElbowAngle) / ArmConstants.elbowArmLength);
      if (thirdSide * thirdSide > ArmConstants.shoulderArmLength * ArmConstants.shoulderArmLength + 
        ArmConstants.elbowArmLength + ArmConstants.elbowArmLength){
        angles[1] = Math.PI - angles[1];
      }
      angles[0] = Math.abs(Math.PI/2 - oppositeElbowAngle - Math.atan(targetY/Math.abs(targetX)));
      if (targetX > 0) angles[0] = -angles[0];
      if (angles[0] > 0){
        angles[1] = -angles[1];
      }
    }   
    return angles;
  }

  public void run(double shoulderPower, double elbowPower){
    elbowMotor.set(elbowPower);
    shoulderMotor.set(shoulderPower);
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
