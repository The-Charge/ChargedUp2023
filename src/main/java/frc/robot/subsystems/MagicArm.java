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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.MagicArmCnsts;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.robotLimit;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/*
 * In this version, counterclockwise rotation is positive and clockwise is
 * Negative. The cartesian coordinate is defined as the origin at the 
 * Shoulder totation axis, negative rotation of shoulder brings it to 
 * The 1st quad. The units are in the MKS system. The team is working 
 * Towards a version that with a different setup that has been agreed 
 * Upon on 3/4/2023. Restricitions: Shoulder and Elbow are in the same
 * Quadrant within some error range.
*/

public class MagicArm extends SubsystemBase {
  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
  private WPI_TalonSRX shldrMtr;
  private WPI_TalonSRX elbowMtr;

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

  // These are the extra variables required to run the MagicArm subsystem
  private double shldrAngl = 0;
  private double elbowAngl = 0;
  private double currentX = 0;
  private double currentY = 0;

  public MagicArm() {
    shldrMtr = new WPI_TalonSRX(ArmConstants.shoulderCAN_ID);
    elbowMtr = new WPI_TalonSRX(ArmConstants.elbowCAN_ID);

    /* Factory default hardware to prevent unexpected behavior */
    shldrMtr.configFactoryDefault();
    elbowMtr.configFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    shldrMtr.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
        MagicArmCnsts.kPIDLoopIdxShldr,
        MagicArmCnsts.kTimeoutMs);
    elbowMtr.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
        MagicArmCnsts.kPIDLoopIdxElbow,
        MagicArmCnsts.kTimeoutMs);

    // Set deadband to super small 0.005 (0.5 %). The default deadband is 0.04 (4 %)
    shldrMtr.configNeutralDeadband(0.005, MagicArmCnsts.kTimeoutMs);
    elbowMtr.configNeutralDeadband(0.005, MagicArmCnsts.kTimeoutMs);

    /* Configure Talon SRX Output and Sensor direction accordingly */
    shldrMtr.setSensorPhase(true);
    shldrMtr.setInverted(false);
    elbowMtr.setInverted(false);

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
    shldrMtr.configMotionCruiseVelocity(100, MagicArmCnsts.kTimeoutMs);
    shldrMtr.configMotionAcceleration(100, MagicArmCnsts.kTimeoutMs);
    shldrMtr.configMotionSCurveStrength(5, MagicArmCnsts.kTimeoutMs);
    elbowMtr.configMotionCruiseVelocity(200, MagicArmCnsts.kTimeoutMs);
    elbowMtr.configMotionAcceleration(100, MagicArmCnsts.kTimeoutMs);
    elbowMtr.configMotionSCurveStrength(3, MagicArmCnsts.kTimeoutMs);

    /* Set the relative sensor according to absolute sensor at neutral on robot boot up */
    int shldrTick = shldrMtr.getSensorCollection().getPulseWidthPosition() % 4096 - 1623;
    if (shldrTick > 2048) {
      shldrTick -= 4096;
    } else if (shldrTick < -2048) {
      shldrTick += 4096;
    }
    shldrMtr.setSelectedSensorPosition(-shldrTick, MagicArmCnsts.kPIDLoopIdxShldr, MagicArmCnsts.kTimeoutMs);

    int elbowTick = (elbowMtr.getSensorCollection().getPulseWidthPosition() % 4096) - 3195; 
    if (elbowTick > 2048) {
      shldrTick -= 4096;
    } else if (elbowTick < -2048) {
      elbowTick += 4096;
    }
    elbowMtr.setSelectedSensorPosition(elbowTick, MagicArmCnsts.kPIDLoopIdxElbow, MagicArmCnsts.kTimeoutMs);

    /* Break mode activation for elbow and shoulder */
    shldrMtr.setNeutralMode(NeutralMode.Brake);
    elbowMtr.setNeutralMode(NeutralMode.Brake);

    // Use motion magic to set the arms in place.
    shldrMtr.set(ControlMode.MotionMagic, -shldrTick);
    elbowMtr.set(ControlMode.MotionMagic, elbowTick);
  }

  public void setCoastMode() {
    elbowMtr.setNeutralMode(NeutralMode.Coast);
    shldrMtr.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Determine whether the (_x,_y) cooridnate is allowed by the game rule and will
   * Not damage the robot.
   * 
   * @param _x
   * @param _y
   * @return True if the cooridnate is allowed, false otherwise.
   */
  public boolean isXYInLimit(double _x, double _y) {
    // Not into the floor
    if (_y < -ArmConstants.shoulderHeight + 0.02) {
      return false;
    }

    // Not above the height limit
    if (_y + ArmConstants.shoulderHeight > robotLimit.height) {
      return false;
    }

    // Not extend >48 inch from the robot
    if (Math.abs(_x) > robotLimit.widthFromCenter) {
      return false;
    }

    // Not below the shoulder mount while inside the robot
    if (Math.abs(_x) < robotLimit.robotLength / 2 && _y < 0) {
      return false;
    }
    return true;
  }
  
  /***
   * Clamp x value within the game rule limit.
   * @param _x
   * @return clamped x value 
   */
  public double getLimitX(double _x) {
    return MathUtil.clamp(_x, -robotLimit.widthFromCenter, robotLimit.widthFromCenter);
  }

  /***
   * Clamp y value within the game rule limit and above floor.  If the arm tip is inside
   * the robot, clamp y between the shoulder height and the height that can be reached if the 
   * shoulder is neutral to eliminate wild shoulder movement.  The later requirement is relaxed if 
   * y is above 20 inches to prevent it from sudden dropping if a hit forced the arm tip inside the 
   * robot limit but the arm tip is already high because it is doing an important task.
   * @param _x
   * @param _y
   * @return clamped y value
   */
  public double getLimitY(double _x, double _y) {
    if (Math.abs(_x) < robotLimit.robotLength / 2 + 0.1 && _y < 0.7) {
      return MathUtil.clamp(_y, 0,
          ArmConstants.shoulderL - (Math.sqrt(ArmConstants.elbowL * ArmConstants.elbowL - _x * _x)));
    } else {
      return MathUtil.clamp(_y, -ArmConstants.shoulderHeight + 0.02, robotLimit.height - ArmConstants.shoulderHeight);
    }
  }
  
  /**
   * Get the (x,y) coordinate of the arm tip given a shoulder angle and an elbow
   * Angle.
   * 
   * @param _shoulderAngle
   * @param _elbowAngle
   * @return A double array, the first element is the x value and the second
   *         Element is the y value.
   */
  public double[] getXY(double _shoulderAngle, double _elbowAngle) {
    double[] xy = new double[2];
    // Find the angle between x axis and the shoulder
    double shoulderHorizen = Math.PI / 2 + _shoulderAngle;

    // Find the angle between x axis and the elbow
    double elbowHorizen = -Math.PI / 2 + _elbowAngle + _shoulderAngle;
    // calculating for the x position
    xy[0] = Math.cos(shoulderHorizen) * ArmConstants.shoulderL + Math.cos(elbowHorizen) * ArmConstants.elbowL;
    // Calculating for the y position
    xy[1] = Math.sin(shoulderHorizen) * ArmConstants.shoulderL + Math.sin(elbowHorizen) * ArmConstants.elbowL;
    return xy;
  }

  /**
   * Determine whether a shoulder and elbow angle pair is allowed by the game
   * Rules and won't damage the robot
   * And also the shoulder angle and the elbow angle is within the range of the
   * Robot design.
   * 
   * @param _shoulderAngle
   * @param _elbowAngle
   * @return True if the cooridnate is allowed, false otherwise.
   */
  public boolean isAngleInLimit(double _shoulderAngle, double _elbowAngle) {
    double[] xy = getXY(_shoulderAngle, _elbowAngle);
    return isXYInLimit(xy[0], xy[1]) && (Math.abs(_shoulderAngle) < robotLimit.shoulderRange) &&
        (Math.abs(_elbowAngle) < robotLimit.elbowRange);
  }

  /**
   * Calculate desired shoulder and elbow angles to put the arm tip at the (_x,_y)
   * Coordinate. It uses the triangle formed by the shoulder axle, the elbow axle,
   * And (_x,_y) as the three triangle vertices and use the law of cosine to solve for the
   * Angles.
   * 
   * @param _x
   * @param _y
   * @return A double array with the first element is the shoulder angle and the
   *         Second element is the elbow angle and
   *         The third element is 1 if a solution can be found and -1 for no
   *         Feasible solution.
   */
  public double[] getAngles(double _x, double _y) {
    double[] angles = new double[3];
    // Avoiding divide by 0 problem
    if (Math.abs(_x) < 0.01) {
      angles[0] = 0;
      angles[1] = 0;
      angles[2] = 1;
      return angles;
    }
    double thirdSide2 = _x * _x + _y * _y;  //square of the third side length
    /**
     * Find the length of the segment connecting (_x,_y) point to the shoulder
     * Axle
     */
    double thirdSide = Math.sqrt(thirdSide2);

    if (thirdSide + ArmConstants.elbowL < ArmConstants.shoulderL ||
        thirdSide > ArmConstants.elbowL + ArmConstants.shoulderL) {
      // No triangle exists
      angles[2] = -1;
    } else {
      double shoulder2 = ArmConstants.shoulderL * ArmConstants.shoulderL; //shoudler square
      double elbow2 = ArmConstants.elbowL * ArmConstants.elbowL; //elbow square
      // Law of cosine to solve for the angle which is created with the third side
      // with the shoulder (A^2+B^2-C^2)/(2*A*B)
      double oppositeElbowAngle = Math.acos((thirdSide2 + shoulder2 - elbow2) / 2 / ArmConstants.shoulderL / thirdSide);

      // 2nd application of law of cosine to solve for the elbow angle
      angles[1] = Math.acos((shoulder2 + elbow2 - thirdSide2) / 2 / ArmConstants.elbowL / ArmConstants.shoulderL);

      // Convert angle from +x axis to the shoulder motor angle
      angles[0] = (Math.PI / 2 - oppositeElbowAngle - Math.atan(_y / Math.abs(_x)));

      if (_x > 0) {
        // The triangle is solved as in the 1st quadrant, now compensate if it is in the 2nd quad.
        angles[0] = -angles[0];
      }else{
        angles[1] = -angles[1];
      }

      if (Math.abs(angles[0]) > robotLimit.shoulderRange || Math.abs(angles[1]) > robotLimit.elbowRange) {
        /**
         * Tell user to ignore solutions because it is outside the range set by the
         * Robot design.
         */
        angles[2] = -1;
      } else {
        // Acceptable solution
        angles[2] = 1;
      }
    }
    return angles;
  }

  /**
   * Run motion magic to set the shoulder and the elbow angle.
   * 
   * @param _shldrAngl
   * @param _elbowAngl
   */
  public void run(double _shldrAngl, double _elbowAngl) {
    shldrMtr.set(ControlMode.MotionMagic, _shldrAngl / ArmConstants.shoulderperMotorTick);
    elbowMtr.set(ControlMode.MotionMagic, _elbowAngl / ArmConstants.elbowperMotorTick);
  }

  /**
   * Run motion matic to set the elbow angle.
   * @param _elbowAngl
   */
  public void runElbow(double _elbowAngl) {
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
    // Makes variables and get elbow and shoulder ticks
    double elbowTicks = elbowMtr.getSelectedSensorPosition();
    double shldrTicks = shldrMtr.getSelectedSensorPosition();

    // Calculates elbow and shoulder angle based on ticks
    elbowAngl = elbowTicks * ArmConstants.elbowperMotorTick;
    shldrAngl = shldrTicks * ArmConstants.shoulderperMotorTick;
    double shoulderHorizen = Math.PI / 2 + shldrAngl;
    double elbowHorizen = -Math.PI / 2 + elbowAngl + shldrAngl;
    // calculating for the x position
    currentX = Math.cos(shoulderHorizen) * ArmConstants.shoulderL + Math.cos(elbowHorizen) * ArmConstants.elbowL;
    // Calculating for the y position
    currentY = Math.sin(shoulderHorizen) * ArmConstants.shoulderL + Math.sin(elbowHorizen) * ArmConstants.elbowL;

    // Displays elbow and shoulder ticks on smart dashboard
    SmartDashboard.putNumber("ElbowEncoder", elbowTicks);
    SmartDashboard.putNumber("ShoulderEncoder", shldrTicks);

    // Displays elbow and shoulder angle and armtip coordinates on smart dashboard
    SmartDashboard.putNumber("currentShoulderAngle", shldrAngl / 3.14 * 180.0);
    SmartDashboard.putNumber("currentElbowAngle", elbowAngl / 3.14 * 180.0);
    SmartDashboard.putNumber("Arm at X", currentX);
    SmartDashboard.putNumber("Arm at Y", currentY);
  }

  /**
   * Get the x value for the arm tip (meters)
   * @return
   */
  public double getX(){
    return currentX;
  }

  /**
   * Get the y value for the arm tip (meters)
   * @return
   */
  public double getY(){
    return currentY;
  }

  /**
   * Return whether the arm tip is inside the robot for claw safety.  A safety margin is put in to account 
   * for armtip motion inbetween a cycle and any potential sensor lag.
   * @return true if the arm tip is inside the robot, false otherwise
   */
  public boolean isArmTipInsideRobotX() {
    if (currentY > 0.5) {
      return false;
    }
    return Math.abs(currentX) < robotLimit.robotLength / 2 + 0.15;
  }

  /**
   * Move the arm tip to the desired (x,y) coordinates. The starting position
   * needs to be close to a neutral position: shoulder up, Elbow down.
   * 
   * @param _x
   * @param _y
   * @return True if a solution is available, otherwise false.
   */
  public boolean moveTowardXYFromNeutral(double _x, double _y) {
    if (isXYInLimit(_x, _y)) { //only moves if the input coordinate is legal
      double[] angles = getAngles(_x, _y);
      if (angles[2] > 0) { // a triangle solution exists
        // Elbow already at target, move shoulder
        if (Math.abs(angles[1] - elbowAngl) < 0.05) {
          run(angles[0], angles[1]);
        } else {
          // Only move the elbow if the arm tip has not cleared the robot,
          if (Math.abs(currentX) < robotLimit.robotLength / 2) {
            run(shldrAngl, angles[1]);
          } else if (currentY < -ArmConstants.shoulderHeight + 0.02) {
            // Move the elbow and raise the shoulder a little if the arm tip is too low.
            run(shldrAngl * 0.99, angles[1]);
          } else {
            // Move both arms if the arm tip has cleared the robot and not on the ground.
            run(angles[0], angles[1]);
          }
        }
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  /**
   * To lower the arm tip to the neutral position: shoulder up, elbow down
   */
  public void moveTowardNeutral() {
    if (Math.abs(shldrAngl) < ArmConstants.shoulderAngleToSafeSwingElbowThrough) {
      run(0, 0);
    }
    else if (Math.abs(elbowAngl) < Units.degreesToRadians(57.5)) { //Not safe to move elbow
      run (0, elbowAngl);
    }
    else {
      run(0,0);
    }
  }

  /**
   * Move the arm tip to the desired (x,y) coordinates.Use moveTOXYFromNeutral if
   * the starting position is close to neutral since That method has less
   * calculations.
   * 
   * @param _x
   * @param _y
   * @return True if a solution is available, otherwise false.
   */
  public boolean moveTowardXY(double _x, double _y) {
    if (isXYInLimit(_x, _y)) {
      double[] angles = getAngles(_x, _y);
      // A solution is found
      if (angles[2] > 0) {
        /**
         * The current and the desired arm tip positions at the same side of the robot
         */
        if (currentX * _x > 0) {
          moveTowardXYFromNeutral(_x, _y);
        } else {
          if (Math.abs(shldrAngl) < ArmConstants.shoulderAngleToSafeSwingElbowThrough) {
            // Optimization to avoid the elbow slow down at the neutral position
            run(0, angles[1]);
          } else {
            // Move to neutral position if optimization cannot be safely performed
            moveTowardNeutral();
          }
        }
        return true;
      } else {
        return false;
      }
    }
    return false;
  }
}
