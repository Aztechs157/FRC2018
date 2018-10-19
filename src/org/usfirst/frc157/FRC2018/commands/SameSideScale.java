
package org.usfirst.frc157.FRC2018.commands;

import org.usfirst.frc157.FRC2018.PID;
import org.usfirst.frc157.FRC2018.Robot;
import org.usfirst.frc157.FRC2018.subsystems.Lift;

import edu.wpi.first.wpilibj.command.Command;

public class SameSideScale extends Command
{
    // same side scale
    public enum autonState
    {
        forward1, turn1, forward2, wait1, back1;
    }

    private autonState state;
    private boolean autonFinished = false;
    private int left = 1;
    private double platPower;
    private double elevatorPower;
    private double platTarget;
    private double elevatorTarget;
    private PID elevatorPID;
    private PID platPID;
    private DriveTarget forward1;
    private GyroTurn turn1;
    private DriveTarget forward2;
    private DriveTarget back1;
    private int waitReps;

    public SameSideScale(boolean left)
    {
        // same side scale
        requires(Robot.drive);
        requires(Robot.grabber);
        requires(Robot.lift);
        state = autonState.forward1;
        platPID = new PID(1, 0, 0, 999999, 999999, 9999999, 99999);
        elevatorPID = new PID(1, 0, 0, 999999, 999999, 9999999, 99999);
        System.out.println("Same side scale got called");
        this.left = (left) ? 1 : -1;
        platTarget = 40.5;
        elevatorTarget = 37.5;
        forward1 = new DriveTarget(198, 0, 3, 5);
        turn1 = new GyroTurn(this.left * 90, 3, 3, 0.4);
        forward2 = new DriveTarget(30, this.left * 45, 3, 3);
        back1 = new DriveTarget(-50, this.left*45, 3, 8);
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
                    Robot.grabber.move(-1);
                    state = autonState.wait1;
                }
                break;
            case forward2:
                moveLift();
                if (forward2.execute())
                {
                    reset();
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
                    Robot.grabber.move(-1);
                    state = autonState.back1;
                    autonFinished = true;
//                    platTarget = 0;
//                    elevatorTarget = 0;
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
        System.out.println("Encoders: "+Robot.drive.getLeftEncoder()+", "+Robot.drive.getRightEncoder());
        Robot.drive.AutoDrive(0, 0);
        Robot.drive.resetLeftEncoder();
        Robot.drive.resetRightEncoder();
    }
}
