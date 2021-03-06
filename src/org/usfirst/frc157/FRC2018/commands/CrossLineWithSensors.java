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
import org.usfirst.frc157.FRC2018.Robot;

import edu.wpi.first.wpilibj.command.Command;
//import org.usfirst.frc157.FRC2018.Robot;

/**
 *
 */
public class CrossLineWithSensors extends Command {
    //cross the line

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    private boolean autonFinished = false;
    private DriveTarget forward1;
    
    public CrossLineWithSensors()
    {
        //middle auton
        requires(Robot.drive);
        forward1 = new DriveTarget(144, 0, 5, 14);
    }
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
       reset();
       System.out.println("CrosslineWithSensors: initialize()"); 
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if (forward1.execute()) {
            autonFinished = true;
            reset();
            System.out.println("CrossLineWithSensors:execute():broken 1");
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return autonFinished;
    } 

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.drive.AutoDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        Robot.drive.AutoDrive(0, 0);
    }
    public void reset() {
        Robot.drive.AutoDrive(0,0);
        Robot.drive.resetLeftEncoder();
        Robot.drive.resetRightEncoder();
   }
}
