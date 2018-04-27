
package org.usfirst.frc157.FRC2018.commands;

import org.usfirst.frc157.FRC2018.PID;
import org.usfirst.frc157.FRC2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class OppositeSideSwitch extends Command
{
    // opposite side switch
    public enum autonState
    {
        forward1, turn1, forward2, turn2, forward3, turn3, forward4, wait1, back1;
    }

    private autonState state;
    private boolean autonFinished = false;
    private int left = 1;
    private double platPower;
    private PID platPID;
    private int platTarget;
    private DriveTarget forward1;
    private GyroTurn turn1;
    private DriveTarget forward2;
    private GyroTurn turn2;
    private DriveTarget forward3;
    private GyroTurn turn3;
    private DriveTarget forward4;
    private DriveTarget back1;
    private int waitReps = 0;

    public OppositeSideSwitch(boolean left)
    {
        // opposite side switch
        requires(Robot.drive);
        requires(Robot.grabber);
        requires(Robot.lift);
        state = autonState.forward1;
        platPID = new PID(1, 0, 0, 999999, 999999, 9999999, 99999);
        System.out.println("Opposite Side Switch got called");
        this.left = (left) ? 1 : -1;
        platTarget = 35;
        forward1 = new DriveTarget(168, 0, 3, 5);
        turn1 = new GyroTurn(this.left * 90, 2, 3, 0.4);
        forward2 = new DriveTarget(179, this.left * 90, 3, 5);
        turn2 = new GyroTurn(this.left*180, 2, 3, 0.4);
        forward3 = new DriveTarget(52, this.left*180, 3, 2);
        turn3 = new GyroTurn(this.left*270, 2, 3, 0.4);
        forward4 = new DriveTarget(28, this.left*270, 3, 2);
        back1 = new DriveTarget(-28, this.left*270, 3, 4);
    }

    @Override
    protected void execute()
    {
        switch (state)
        {
            case forward1:
                if (forward1.execute()) {
                    reset();
                    state = autonState.turn1;
 
                }
                break;
            case turn1:
                if (turn1.execute()) {
                    reset();
                    state = autonState.forward2;


                }
                break;
            case forward2:
                if (forward2.execute()) {
                    reset();
                    state = autonState.turn2;


                }
                break;
            case turn2:
                if (turn2.execute()) {
                    reset();
                    state = autonState.forward3;
                }
                break;
            case forward3:
                if (forward3.execute()) {
                    reset();
                    state = autonState.turn3;
                }
                break;
            case turn3:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                if (turn3.execute()) {
                    reset();
                    state = autonState.forward4;
                }
                break;
            case forward4:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                if (forward4.execute()) {
                    reset();
                    //Robot.grabber.move(-1);
                    state = autonState.wait1;
                }
                break;
            case wait1:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                Robot.grabber.move(-1);
                if (waitReps>50) {
                    reset();
                    state = autonState.back1;
                    waitReps = 0;
                }
                else {
                    waitReps++;
                }
                break;
            case back1:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                if (back1.execute()) {
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
        Robot.drive.AutoDrive(0, 0);
        Robot.drive.resetLeftEncoder();
        Robot.drive.resetRightEncoder();
    }
}
