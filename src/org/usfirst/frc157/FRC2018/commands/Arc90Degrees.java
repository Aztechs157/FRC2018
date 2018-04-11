
package org.usfirst.frc157.FRC2018.commands;

import org.usfirst.frc157.FRC2018.PID;
import org.usfirst.frc157.FRC2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class Arc90Degrees extends Command
{
    // opposite side switch
    public enum autonState
    {
        arc1;
    }

    private autonState state;
    private boolean autonFinished = false;
    private int left = 1;
    private double platPower;
    private PID platPID;
    private int platTarget;
    private DriveTarget forward1;
    private Ellipse arc1;
    private DriveTarget forward2;
    private Ellipse arc2;
    private DriveTarget forward3;
    private Ellipse arc3;
    private DriveTarget forward4;
    private DriveTarget back1;
    private int waitReps = 0;
    private double encoder;

    public Arc90Degrees(boolean left)
    {
        // opposite side switch
        requires(Robot.drive);
        requires(Robot.grabber);
        requires(Robot.lift);
        state = autonState.arc1;
        platPID = new PID(1, 0, 0, 999999, 999999, 9999999, 99999);
        System.out.println("Opposite Side Switch got called");
        this.left = (left) ? 1 : -1;
        arc1 = new Ellipse(96, 96, this.left*-1, 150, 0, 5, 10);
    }

    @Override
    protected void execute()
    {
        switch (state)
        {
            case arc1:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                encoder = (Robot.drive.getRightEncoder() + Robot.drive.getLeftEncoder()) / 2.0;
                if (arc1.execute()) {
                    autonFinished = true;
                }
                break;
        }
    }

    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return autonFinished;
    }

    @Override
    protected void end()
    {
        Robot.lift.hold();
    }

    @Override
    protected void interrupted()
    {
        Robot.lift.hold();
    }
    public void reset()
    {
       // Robot.drive.AutoDrive(0, 0);
        Robot.drive.resetLeftEncoder();
        Robot.drive.resetRightEncoder();
    }
}
