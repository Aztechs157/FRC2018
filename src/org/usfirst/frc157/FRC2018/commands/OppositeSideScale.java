
package org.usfirst.frc157.FRC2018.commands;

import org.usfirst.frc157.FRC2018.PID;
import org.usfirst.frc157.FRC2018.Robot;
import org.usfirst.frc157.FRC2018.commands.SameSideScale.autonState;

import edu.wpi.first.wpilibj.command.Command;

public class OppositeSideScale extends Command
{
    // opposite side switch
    public enum autonState
    {
        forward1, turn1, forward2, turn2, forward3, wait1, back1;
    }

    private autonState state;
    private boolean autonFinished = false;
    private int left = 1;
    private double platPower;
    private PID platPID;
    private int platTarget;
    private PID elevatorPID;
    private int elevatorTarget;
    private double elevatorPower;
    private DriveTarget forward1;
    private GyroTurn turn1;
    private DriveTarget forward2;
    private GyroTurn turn2;
    private DriveTarget forward3;
    private DriveTarget back1;
    private int waitReps;

    public OppositeSideScale(boolean left)
    {
        // opposite side switch
        requires(Robot.drive);
        state = autonState.forward1;
        platPID = new PID(1, 0, 0, 999999, 999999, 9999999, 99999);
        elevatorPID = new PID(1, 0, 0, 999999, 999999, 9999999, 99999);
        elevatorTarget = 0;
        System.out.println("Opposite Side Scale got called");
        this.left = (left) ? 1 : -1;
        platTarget = 30;
        forward1 = new DriveTarget(204, 0, 3, 5);
        turn1 = new GyroTurn(this.left * 90, 2, 3, 0.4);
        forward2 = new DriveTarget(195, this.left * 90, 3, 5);
        turn2 = new GyroTurn(0, 2, 3, 0.4);
        forward3 = new DriveTarget(55, 0, 3, 2);
        back1 = new DriveTarget(-55, 0, 3, 3);
        waitReps = 0;
    }

    @Override
    protected void execute()
    {
        switch (state)
        {
            case forward1:
                moveLift();
                if (forward1.execute())
                {
                    reset();
                    state = autonState.turn1;
                }
                break;
            case turn1:
                moveLift();
                if (turn1.execute())
                {
                    reset();
                    state = autonState.forward2;
                }
                break;
            case forward2:
                moveLift();
                if (forward2.execute())
                {
                    reset();
                    elevatorTarget = 35;
                    state = autonState.turn2;
                }
                break;
            case turn2:
                moveLift();
                if (turn2.execute())
                {
                    reset();
                    state = autonState.forward3;
                }
                break;
            case forward3:
                moveLift();
                if (forward3.execute())
                {
                    reset();
                    Robot.grabber.move(1);
                    state = autonState.wait1;
                }
                break;
            case back1:
                moveLift();
                if (back1.execute())
                {
                    reset();
                    Robot.grabber.move(0);
                    autonFinished = true;
                }
                break;
            case wait1:
                moveLift();
                if (waitReps>30) {
                    reset();
                    Robot.grabber.move(1);
                    elevatorTarget = 0;
                    platTarget = 0;
                    state = autonState.back1;
                    waitReps = 0;
                }
                else {
                    waitReps++;
                }
                break;
        }
    }

    public void moveLift() {
        platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
        Robot.lift.setPlat(platPower);
        elevatorPower = elevatorPID.pidCalculate(elevatorTarget, Robot.lift.getStageEncoder());
        Robot.lift.setStage(-elevatorPower);
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
