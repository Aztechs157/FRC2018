// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc157.FRC2018.commands;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc157.FRC2018.OI;
import org.usfirst.frc157.FRC2018.Robot;
import org.usfirst.frc157.FRC2018.subsystems.Lift;

/**
 *
 */
public class DriveLiftWithSticks extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    boolean running = false;
    double direction = 0;
    double endPos = 0;
    final double TOLERANCE = 0.1;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public DriveLiftWithSticks() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.lift);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        direction = 0;
        if (Math.abs(Robot.oi.getopBox().getRawAxis(OI.JoyY))>TOLERANCE)
        {
            direction = Robot.oi.getopBox().getRawAxis(OI.JoyY);
        }
        else if (Math.abs(Robot.oi.getopBox().getRawAxis(OI.JoyX))>TOLERANCE)
        {
            direction = -Robot.oi.getopBox().getRawAxis(OI.JoyX);
        }
        else
        {
            Robot.lift.stop();
        }
        SmartDashboard.putNumber("joystick", direction);
        if (direction > TOLERANCE)
        {
            Robot.lift.move(Lift.direction.UP, Math.abs(Robot.oi.getopBox().getRawAxis(OI.JoyX))>TOLERANCE, 1);
        }
        else if (direction < -TOLERANCE)
        {
            //System.out.println("down");
            Robot.lift.move(Lift.direction.DOWN, Math.abs(Robot.oi.getopBox().getRawAxis(OI.JoyX))>TOLERANCE, -0.75);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.lift.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        Robot.lift.stop();
    }
}
