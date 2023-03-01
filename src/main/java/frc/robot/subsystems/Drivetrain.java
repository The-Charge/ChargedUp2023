// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


import frc.robot.commands.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SerialPort.Port;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Drivetrain extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private WPI_TalonSRX leftFrontMotor;
private WPI_TalonSRX leftRearMotor;
private MotorControllerGroup left;
private WPI_TalonSRX rightFrontMotor;
private WPI_TalonSRX rightRearMotor;
private MotorControllerGroup right;
private Solenoid shiftSpeed;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private DifferentialDrive differentialDrive;
    private AHRS navx;
    private boolean isReversed = false, isHalfSpeed = false, isQuarterSpeed = false;

    /**
    *
    */
    public Drivetrain() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
leftFrontMotor = new WPI_TalonSRX(16);
 
 

leftRearMotor = new WPI_TalonSRX(17);
 
 

left = new MotorControllerGroup(leftFrontMotor, leftRearMotor  );
 addChild("left",left);
 

rightFrontMotor = new WPI_TalonSRX(3); //real 3 //spare 1
 
 

rightRearMotor = new WPI_TalonSRX(2);
 
 

right = new MotorControllerGroup(rightFrontMotor, rightRearMotor  );
 addChild("right",right);
 

shiftSpeed = new Solenoid(1, PneumaticsModuleType.REVPH, 1);
 addChild("shiftSpeed", shiftSpeed);
 


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    navx = new AHRS(Port.kUSB);
    differentialDrive = new DifferentialDrive(left, right);

    leftFrontMotor.setInverted(false);
    leftFrontMotor.setNeutralMode(NeutralMode.Coast);

    leftRearMotor.setInverted(false);
    leftRearMotor.setNeutralMode(NeutralMode.Coast);

    //Right side inverted
    rightFrontMotor.setInverted(true);
    rightFrontMotor.setNeutralMode(NeutralMode.Coast);

    rightRearMotor.setInverted(true);
    rightRearMotor.setNeutralMode(NeutralMode.Coast);

    rightRearMotor.follow(rightFrontMotor);
    leftRearMotor.follow(leftFrontMotor);

    SmartDashboard.putBoolean("HalfSpeed", isHalfSpeed);
    SmartDashboard.putBoolean("QuarterSpeed", isQuarterSpeed);
    SmartDashboard.putBoolean("Reversed", isReversed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
/* Display 6-axis Processed Angle Data */
SmartDashboard.putBoolean("IMU_Connected", navx.isConnected());
SmartDashboard.putNumber("IMU_Yaw", navx.getYaw());
SmartDashboard.putNumber("IMU_Pitch", navx.getPitch());

// displays encoder ticks
SmartDashboard.putNumber("Left Encoder", getLeftEncoder());
SmartDashboard.putNumber("Right Encoder", getRightEncoder());

SmartDashboard.putNumber("Get Heading", getHeading());

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void run(double l, double r) {
      // runs numbers from joystick y axis
      if (isReversed) {
        l *= -1;
        r *= -1;
      }
      
      if (isQuarterSpeed) {
        l *= 0.25;
        r *= 0.25;
      }else if (isHalfSpeed) {
        l*= 0.5;
        r *= 0.5;}
      differentialDrive.tankDrive(l, r);
    }
    public double getLeftEncoder() {
      return leftFrontMotor.getSelectedSensorPosition();
    }
  
    public double getRightEncoder() {
      return rightRearMotor.getSelectedSensorPosition();
    }

    public void setBrake() {
      leftFrontMotor.setNeutralMode(NeutralMode.Brake);
      leftRearMotor.setNeutralMode(NeutralMode.Brake);
  
      rightFrontMotor.setNeutralMode(NeutralMode.Brake);
      rightRearMotor.setNeutralMode(NeutralMode.Brake);
    }
  
    public void setCoast() {
      leftFrontMotor.setNeutralMode(NeutralMode.Coast);
      leftRearMotor.setNeutralMode(NeutralMode.Coast);
  
      rightFrontMotor.setNeutralMode(NeutralMode.Coast);
      rightRearMotor.setNeutralMode(NeutralMode.Coast);
    }

    public double getHeading() {
      return navx.getRotation2d().getDegrees();
    }
    
    public void resetEncoders() {
      leftFrontMotor.setSelectedSensorPosition(0);
      rightRearMotor.setSelectedSensorPosition(0);
    }

    public void shiftHigh() {
      shiftSpeed.set(true);
    }

    public void shiftLow() {
      shiftSpeed.set(false);
    }

    public void setReversed() {
      isReversed = !isReversed;
    }
    public void setHalfSpeed() {
      isHalfSpeed = !isHalfSpeed;
    }
    public void setQuarterSpeed() {
      isQuarterSpeed = !isQuarterSpeed;
    }
}

