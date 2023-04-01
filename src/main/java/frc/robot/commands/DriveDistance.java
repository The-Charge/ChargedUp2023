// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SysIDConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drivetrain m_drivetrain;
    private final double m_power;
    private final double m_offset;
    private final double m_ticks;
    private double startTick;
    private double controllablePowerRange;
    private double stallPower = 0.34;
    public double drivePower;
    private double ticksToTravel = 0;
    private double ticksTravelled = 0;
    /**
     * Drive at set power and set offset (clockwise +, counterclockwise -) until last meter
     * then slow down until distance is reached
     * @param subsystem The subsystem used by this command.
     * @param _power Driving power
     * @param _headingOffset Angles in degrees 
     */
    public DriveDistance(Drivetrain subsystem, double _power, double headingOffset,
            double distanceMeters) {
        m_ticks = distanceMeters*SysIDConstants.ticksPerMeter;
        controllablePowerRange = Math.abs(_power) - stallPower;
        if (_power < 0){
          controllablePowerRange = -controllablePowerRange;
          stallPower = -stallPower;
        }
        m_drivetrain = subsystem;
        m_offset = headingOffset;
        m_power = _power;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTick = m_drivetrain.getLeftEncoder();
        ticksToTravel = m_ticks;
        drivePower = m_power;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double headingPower = (m_drivetrain.getHeading() + m_offset) * AutoConstants.headingGain;
        ticksTravelled = Math.abs(m_drivetrain.getLeftEncoder() - startTick);
        ticksToTravel = m_ticks - ticksTravelled;
        if (ticksToTravel < SysIDConstants.ticksPerMeter){
          drivePower = ticksToTravel / SysIDConstants.ticksPerMeter * controllablePowerRange + stallPower;     
        }
        m_drivetrain.run(drivePower + headingPower, drivePower - headingPower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.run(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ticksToTravel < 1.0;
    }
} 