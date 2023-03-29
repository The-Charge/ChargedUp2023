package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveOverDistance extends CommandBase {
   // @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drivetrain m_drivetrain;
    private double m_speed;
    private final double m_stopPitch;
    private double thisPitch;
    private double m_offset = 0;
    private int status = 0;
    public double timeoutMS = 8000;
    public long currentTime;
    public double distance, startTick, startSpeed, m_ticks;

    /**
     * 0 is flat starting state, 1 for up, 2 for flat top, 3 for down, 4 for flat
     * end.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveOverDistance(Drivetrain subsystem, double speed, double stopPitch, double headingOffset,
            double distanceMeters) {
        m_ticks = distanceMeters * 18000;
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
            if (Math.abs(thisPitch) < 2) {
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
                SmartDashboard.putNumber("StartTick", startTick);
            }
            m_speed = (m_ticks - Math.abs(m_drivetrain.getLeftEncoder() - startTick)) / m_ticks * startSpeed;
            SmartDashboard.putNumber("Travel dist", Math.abs(m_drivetrain.getLeftEncoder() - startTick));
            SmartDashboard.putNumber("Speed", m_speed);
        }
        m_drivetrain.run(m_speed + thisHeading, m_speed - thisHeading);
        SmartDashboard.putNumber("Status", status);
        SmartDashboard.putNumber("Left Encoder", m_drivetrain.getLeftEncoder());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.run(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(startTick) > 1) {
            SmartDashboard.putBoolean("driveOverEnded", Math.abs(m_drivetrain.getLeftEncoder() - startTick) > m_ticks * 0.575);
            return Math.abs(m_drivetrain.getLeftEncoder() - startTick) > m_ticks * 0.575;
        }
        return false;
    }
} 