
package org.usfirst.frc157.FRC2018.commands;

import org.usfirst.frc157.FRC2018.PID;
import org.usfirst.frc157.FRC2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class SameSideSwitch extends Command
{
    // same side switch
    public enum autonState
    {
        forward1, turn1, forward2;
    }

    private autonState state;
    private int left = 1;
    private PID platPID;
    private int platTarget;
    private double platPower;
    private boolean autonFinished;
    private DriveTarget forward1;
    private GyroTurn turn1;
    private DriveTarget forward2;

    public SameSideSwitch(boolean left)
    {
        // same side switch
        requires(Robot.drive);
        autonFinished = false;
        state = autonState.forward1;
        platPID = new PID(1, 0, 0, 999999, 999999, 9999999, 99999);
        System.out.println("Same side switch got called");
        this.left = (left) ? 1 : -1;
        platTarget = 30;
        forward1 = new DriveTarget(130, 0, 3, 4);
        turn1 = new GyroTurn(this.left*90, 3, 3, 0.4);
        forward2 = new DriveTarget(30, this.left*90, 3, 4);
    }

    @Override
    protected void execute()
    {
        switch (state)
        {
            case forward1:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                if (forward1.execute()) {
                    state = autonState.turn1;
                    reset();
                }
                break;
            case turn1:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                 if (turn1.execute()) {
                     state = autonState.forward2;
                     reset();
                 }
                break;
            case forward2:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                if (forward2.execute()) {
                     Robot.grabber.move(1);
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

    public void reset()
    {
        Robot.drive.AutoDrive(0, 0);
        Robot.drive.resetLeftEncoder();
        Robot.drive.resetRightEncoder();
    }
}
