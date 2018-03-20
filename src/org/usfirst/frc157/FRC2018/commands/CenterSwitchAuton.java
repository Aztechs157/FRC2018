
package org.usfirst.frc157.FRC2018.commands;

import org.usfirst.frc157.FRC2018.PID;
import org.usfirst.frc157.FRC2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class CenterSwitchAuton extends Command
{
	//middle auton
    public enum autonState
    {
        forward1, turn1, forward2, turn2, forward3
    }

    private autonState state;
    private boolean autonFinished = false;
    private int left = 1;
    private double platPower;
    private double platTarget;
    private PID platPID;
    private DriveTarget forward1;
    private GyroTurn turn1;
    private DriveTarget forward2;
    private GyroTurn turn2;
    private DriveTarget forward3;

    public CenterSwitchAuton(boolean left)
    {
        //middle auton
        requires(Robot.drive);
        state = autonState.forward1;
        platPID = new PID(1, 0, 0, 999999, 999999, 9999999, 99999);
        System.out.println("Middle Switch got instantiated");
        this.left = (left)? 1: -1;
        platTarget = 30;

        forward1 = new DriveTarget(44, 0, 3, 3);
        turn1 = new GyroTurn(this.left*-90, 3, 3, 0.4);
        forward2 = new DriveTarget(((this.left == 1)? 54: 44), this.left*-90, 3, 3);
        turn2 = new GyroTurn(0, 3, 3, 0.4);
        forward3 = new DriveTarget(51, 0, 3, 3);
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
                     state = autonState.turn2;
                     reset();
                 }
                break;
            case turn2:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                if (turn2.execute()) {
                    state = autonState.forward3;
                    reset();
                }
                break;
            case forward3:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                if (forward3.execute()) {
                    autonFinished = true;
                    reset();
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
    public void reset() {
        Robot.drive.AutoDrive(0,0);
        Robot.drive.resetLeftEncoder();
        Robot.drive.resetRightEncoder();
    }
}
