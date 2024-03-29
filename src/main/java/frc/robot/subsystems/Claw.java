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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

public class Claw extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private Solenoid clawSolenoid;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public boolean isOpen = false;

    public Claw() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        clawSolenoid = new Solenoid(1, PneumaticsModuleType.REVPH, 2);
        addChild("clawSolenoid", clawSolenoid);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Claw isOpened", isOpen);
    }

    // This method will be called once per scheduler run when in simulation
    @Override
    public void simulationPeriodic() {

    }

    // Put methods for controlling this subsystem here. Call these from Commands.
    public void openClaw(boolean force) {
        isOpen = true;
        if (force) {
            clawSolenoid.set(true);
        }
    }

    public void closeClaw(boolean force) {
        isOpen = false;
        if (force) {
            clawSolenoid.set(false);
        }
    }

    public void moveClaw(boolean insideRobot) {
        /**
         * Only case where claw should open is when arm is not within robot bounds and
         * Has been opened up. Otherwise it will remain closed
         */
        if (!insideRobot && isOpen) {
            clawSolenoid.set(true);
        } else {
            clawSolenoid.set(false);
        }
    }

    /*
     * Force methods for autonomous.
     * 
     */
    public void forceOpenClaw() {
        clawSolenoid.set(true);
    }

    public void forceCloseClaw() {
        clawSolenoid.set(false);
    }
}
