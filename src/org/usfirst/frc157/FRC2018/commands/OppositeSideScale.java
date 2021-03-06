
package org.usfirst.frc157.FRC2018.commands;

import org.usfirst.frc157.FRC2018.PID;
import org.usfirst.frc157.FRC2018.Robot;
import org.usfirst.frc157.FRC2018.commands.SameSideScale.autonState;
import org.usfirst.frc157.FRC2018.subsystems.Lift;

import edu.wpi.first.wpilibj.command.Command;

public class OppositeSideScale extends Command
{
    // opposite side switch
    public enum autonState
    {
        forward1, turn1, forward2, turn2, forward3, turn3, wait1, back1;
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
    private GyroTurn turn3;
    private DriveTarget back1;
    private int waitReps;

    public OppositeSideScale(boolean left)
    {
        // opposite side switch
        requires(Robot.drive);
        requires(Robot.grabber);
        requires(Robot.lift);
        //state = autonState.forward1;
        state = autonState.forward1;
        platPID = new PID(1, 0, 0, 999999, 999999, 9999999, 99999);
        elevatorPID = new PID(1, 0, 0, 999999, 999999, 9999999, 99999);
        elevatorTarget = 0;
        System.out.println("Opposite Side Scale got called");
        this.left = (left) ? 1 : -1;
        platTarget = 30;
        forward1 = new DriveTarget(170, 0, 3, 5);
        turn1 = new GyroTurn(this.left * 90, 2, 3, 0.4);
        forward2 = new DriveTarget(187, this.left * 90, 3, 5); //drive 20" farther
        turn2 = new GyroTurn(this.left*-30, 2, 3, 0.4); //turn to 30 instead of 0
        forward3 = new DriveTarget(68, this.left*-30, 3, 2, true, true);
        turn3 = new GyroTurn(this.left * -90, 2, 3, 0.4); //no longer called
        back1 = new DriveTarget(-20, 0, 3, 3);
        waitReps = 0;
    }
    
    @Override
    protected void initialize()
    {
        reset();
    }

    @Override
    protected void execute()
    {
        switch (state)
        {
            case forward1:
                //moveLift();
                if (forward1.execute())
                {
                    reset();
                    state = autonState.turn1;
                }
                break;
            case turn1:
                //moveLift();
                if (turn1.execute())
                {
                    reset();
                    state = autonState.forward2;
                }
                break;
            case forward2:
                //moveLift();
                if (forward2.execute())
                {
                    System.out.println("Forward 2");
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
                    System.out.println("rubber ducky had a  good time");
                    reset();
                    //Robot.grabber.move(-1);
                    state = autonState.wait1;
                }
                break;
            case turn3:
                moveLift();
                if (turn3.execute())
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
                Robot.grabber.move(-1);
                if (waitReps>30) {
                    reset();
//                    elevatorTarget = 0;
//                    platTarget = 0;
                   // state = autonState.back1;
                    autonFinished = true;
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
        // TODO Auto-generated method stub

        Lift.platLast = Robot.lift.getPlatEncoder();
        Lift.stageLast = Robot.lift.getStageEncoder();
    }

    public void reset()
    {
        Robot.drive.AutoDrive(0, 0);
        Robot.drive.resetLeftEncoder();
        Robot.drive.resetRightEncoder();
    }
}
