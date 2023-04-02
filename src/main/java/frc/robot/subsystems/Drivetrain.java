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

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SerialPort.Port;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

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

  // These are the extra variables required to run the DriveTrain subsystem
  private final static double kNeutralDeadband = 0.001;
  private DifferentialDrive differentialDrive;
  private AHRS navx;
  private boolean isReversed = false, isHalfSpeed = false, isQuarterSpeed = false;
  private double pitch = 0;
  private double gyroOffset;
  private double pitchOffset;
  private boolean voltNegative;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  private final double hundredMillisecondsToSeconds = 10;

  public Drivetrain() {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    leftFrontMotor = new WPI_TalonSRX(Constants.DriveConstants.kLeftMotor1Port);

    leftRearMotor = new WPI_TalonSRX(Constants.DriveConstants.kLeftMotor2Port);

    left = new MotorControllerGroup(leftFrontMotor, leftRearMotor);
    addChild("left", left);

    rightFrontMotor = new WPI_TalonSRX(2);

    rightRearMotor = new WPI_TalonSRX(3);

    right = new MotorControllerGroup(rightFrontMotor, rightRearMotor);
    addChild("right", right);

    shiftSpeed = new Solenoid(1, PneumaticsModuleType.REVPH, 1); // TODO change to revph
    addChild("shiftSpeed", shiftSpeed);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    navx = new AHRS(Port.kUSB);
    differentialDrive = new DifferentialDrive(left, right);

    leftFrontMotor.setSensorPhase(true);
    leftFrontMotor.setInverted(false);
    leftRearMotor.setInverted(false);

    // Right side inverted
    rightFrontMotor.setSensorPhase(true);
    rightFrontMotor.setInverted(true);
    rightRearMotor.setInverted(true);

    setBrake();

    rightRearMotor.follow(rightFrontMotor);
    leftRearMotor.follow(leftFrontMotor);

    SmartDashboard.putBoolean("HalfSpeed", isHalfSpeed);
    SmartDashboard.putBoolean("QuarterSpeed", isQuarterSpeed);
    SmartDashboard.putBoolean("Reversed", isReversed);
    Boolean isRed = SmartDashboard.getBoolean("Red Alliance", true);
    SmartDashboard.putBoolean("Red Alliance", isRed);
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()),
        getLeftEncoderDistance(),
        getRightEncoderDistance());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /* Display 6-axis Processed Angle Data */
    pitch = -(navx.getPitch() - pitchOffset);

    SmartDashboard.putBoolean("IMU_Connected", navx.isConnected());
    //SmartDashboard.putNumber("IMU_Yaw", navx.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", pitch);

     //Displays encoder ticks
     SmartDashboard.putNumber("Left Encoder", getLeftEncoder());
     SmartDashboard.putNumber("Right Encoder", getRightEncoder());
     

    // Displays distance based on encoders
    SmartDashboard.putNumber("Left Distance", getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Distance", getRightEncoderDistance());

    // Displays the corrected angle of the robot
    SmartDashboard.putNumber("Get Heading", getHeading());
    //SmartDashboard.putString("Pose", getPose().toString());
    m_odometry.update(Rotation2d.fromDegrees(getHeading()),
        getLeftEncoderDistance(),
        getRightEncoderDistance());
    SmartDashboard.putBoolean("Reversed", isReversed);
  }

  // This method will be called once per scheduler run when in simulation
  @Override
  public void simulationPeriodic() {
  }

  /**
   * Put methods for controlling this subsystem Here.
   * Call these from Commands.
   */
  public void initializeMotors() {
    // Config deadband. Called initializeMotors() in initialize in TankDrive.
    TalonSRXConfiguration _leftConfig = new TalonSRXConfiguration();
    TalonSRXConfiguration _rightConfig = new TalonSRXConfiguration();
    _rightConfig.neutralDeadband = kNeutralDeadband;
    _leftConfig.neutralDeadband = kNeutralDeadband;

    leftFrontMotor.configNeutralDeadband(0.08);
    rightFrontMotor.configNeutralDeadband(0.08);

  }

  // Based off joystick Y-axis
  public void run(double l, double r) {
    // Reversed and Multiplier logic
    if (isReversed) {
      double temp = l;
      l = -1 * r;
      r = -1 * temp;
    }

    if (isQuarterSpeed) {
      l *= 0.5;
      r *= 0.5;
    } else if (isHalfSpeed) {
      l *= 0.75;
      r *= 0.75;
    }
    differentialDrive.tankDrive(l, r);
  }

  public boolean reversed(){
    return isReversed;
  }
  public boolean atHalfSpeed(){
    return isHalfSpeed;
  }

  public boolean atQuarterSpeed(){
    return isQuarterSpeed;
  }

  public void runArcade(double f, double r) {
    if (f > 0.6) {
      f = 0.6;
    } else if (f < 0.4 && f > 0) {
      f = 0.4;
    } else if (f > -0.4 && f < 0) {
      f = -0.4;
    }

    if (r > 0.7) {
      r = 0.7;
    } else if (r < -0.7) {
      r = -0.7;
    } else if (r < 0.5 && r > 0) {
      r = 0.5;
    } else if (r > -0.5 && r < 0) {
      r = -0.5;
    }
    differentialDrive.arcadeDrive(-1 * f, -1 * r);
  }

  /**
   * public void runVelocity(double l, double r) {
   * if (isReversed) {
   * l *= -1 * r;
   * r *= -1 * l;
   * }
   * 
   * if (isQuarterSpeed) {
   * l *= 0.5;
   * r *= 0.5;
   * } else if (isHalfSpeed) {
   * l *= 0.75;
   * r *= 0.75;
   * }
   * setControlMode(ControlMode.Velocity);
   * left.set(l * DriveConstants.MAX_VELOCITY);
   * right.set(r * DriveConstants.MAX_VELOCITY);
   * }
   *
   * public void setControlMode(ControlMode mode) {
   * leftFrontMotor.set(mode, 0);
   * leftRearMotor.set(mode, 0);
   * rightFrontMotor.set(mode, 0);
   * rightRearMotor.set(mode, 0);
   * }
   */
  public void setVolt() {
    voltNegative = !voltNegative;
  }

  public double getLeftEncoder() {
    return leftFrontMotor.getSelectedSensorPosition();
  }

  public double getRightEncoder() {
    return rightFrontMotor.getSelectedSensorPosition();
  }

  public double getPitch() {
    return pitch;
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

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getLeftEncoderVelocity(),
        getRightEncoderVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    if (voltNegative) {
      double temp = leftVolts;
      leftVolts = -rightVolts;
      rightVolts = -temp;
    }
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
  }

  public void resetOdometry(Pose2d pose) {
    zeroHeading();
    resetEncoders();
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getHeading()), getLeftEncoderDistance(), getRightEncoderDistance(), pose);
  }

  public double getHeading() {
    return navx.getRotation2d().getDegrees() - gyroOffset;
  }

  public void zeroHeading() {
    // navx.reset();
    gyroOffset = navx.getAngle();
  }

  public void resetPitch() {
    pitchOffset = navx.getPitch();
  }

  public void resetHeading() {
    gyroOffset = navx.getRotation2d().getDegrees();
  }

  public void resetEncoders() {
    leftFrontMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
  }

  public double getLeftEncoderVelocity() {
    return leftFrontMotor.getSelectedSensorVelocity()
        * hundredMillisecondsToSeconds
        / Constants.SysIDConstants.leftEncoderTicksPerMeter;
  }

  public double getRightEncoderVelocity() {
    return rightFrontMotor.getSelectedSensorVelocity()
        * hundredMillisecondsToSeconds
        / Constants.SysIDConstants.rightEncoderTicksPerMeter;
  }

  public double getLeftEncoderDistance() {
    return getLeftEncoder() / Constants.SysIDConstants.leftEncoderTicksPerMeter;
  }

  public double getRightEncoderDistance() {
    return getRightEncoder() / Constants.SysIDConstants.rightEncoderTicksPerMeter;
  }

  public void shiftHigh() {
    shiftSpeed.set(false);
  }

  public void shiftLow() {
    shiftSpeed.set(true);
  }

  public void setReversed() {
    isReversed = !isReversed;
  }

  public void setHalfSpeed() {
    isHalfSpeed = !isHalfSpeed;
    // Is not QuarterSpeed
    if (isQuarterSpeed && isHalfSpeed) {
      isQuarterSpeed = false;
    }
  }

  public void setQuarterSpeed() {
    isQuarterSpeed = !isQuarterSpeed;
    // Is not HalfSpeed
    if (isHalfSpeed && isQuarterSpeed) {
      isHalfSpeed = false;
    }
  }

  public void setFullSpeed(){
    isHalfSpeed = false;
    isQuarterSpeed = false;
  }

  public void setControlMode(ControlMode mode) {
    leftFrontMotor.set(mode, 0);
    rightFrontMotor.set(mode, 0);
  }
}
