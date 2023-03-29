package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveOverDistance extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drivetrain m_drivetrain;
    private double m_speed;
    private final double m_stopPitch;
    private double thisPitch;
    private double m_offset = 0;
    private int status = 0;
    public double timeoutMS = 8000;
    public long currentTime;
    public double distance, startTick, startSpeed, m_ticks ;
    public static final double stallPower = 0.34;
    private double ticksToTravel = 0;
    private double ticksTravlled = 0;
    /**
     * 0 is flat starting state, 1 for up, 2 for flat top, 3 for down, 4 for flat
     * end.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveOverDistance(Drivetrain subsystem, double speed, double stopPitch, double headingOffset,
            double distanceMeters) {
        ticksToTravel = distanceMeters*Constants.SysIDConstants.ticksPerMeter;
        /* We will use fraction of ticks left to travel multiplied by speed as the power for driving
         * motors.  In order to keep the power above stallPower, we need to increase the target ticks
         * so that when ticksToTravel equals ticksTravelled, the power is at stallPower
         */
        double fractionPowerBeforeStall = (Math.abs(speed)-stallPower)/Math.abs(speed);
        m_ticks = ticksToTravel/fractionPowerBeforeStall;
        m_drivetrain = subsystem;
        m_stopPitch = stopPitch;
        m_offset = headingOffset;
        m_speed = speed;
        startSpeed = speed;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        status = 0;
        startTick = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        thisPitch = m_drivetrain.getPitch();
        double thisHeading = (m_drivetrain.getHeading() + m_offset) * AutoConstants.headingGain;

        if (status == 0) {
            if (Math.abs(thisPitch) > m_stopPitch) {
                status = 1;
            }
        } else if (status == 1) {
            if (Math.abs(thisPitch) < 4) {
                status = 2;
                m_speed = m_speed * 0.8;
            }
        } else if (status == 2) {
            if (Math.abs(thisPitch) > m_stopPitch * 0.8) {
                status = 3;
                m_speed = m_speed * .7;
            }
        }
        else if (status == 3) {
            if (Math.abs(thisPitch) < 4) {
                status = 4;
            }
        }
        else if (status == 4) {
            if (Math.abs(startTick) < 1) {
                startTick = m_drivetrain.getLeftEncoder();
            }
            ticksTravlled = Math.abs(m_drivetrain.getLeftEncoder() - startTick);
            m_speed = (m_ticks - ticksTravlled) / m_ticks * startSpeed;
        }
        m_drivetrain.run(m_speed + thisHeading, m_speed - thisHeading);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.run(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ticksTravlled > ticksToTravel;
    }
} 