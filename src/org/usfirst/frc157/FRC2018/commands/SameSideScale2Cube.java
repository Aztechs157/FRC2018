
package org.usfirst.frc157.FRC2018.commands;

import org.usfirst.frc157.FRC2018.PID;
import org.usfirst.frc157.FRC2018.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

public class SameSideScale2Cube extends Command
{
    // same side scale
    public enum autonState
    {
        forward1, turn1, forward2, wait1, back1, arc1, arc2, forward3, raisePlat;
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
    private Ellipse arc1;
    private Ellipse arc2;
    private DriveTarget forward3;
    private GyroTurn turn3;
    private int waitReps;

    public SameSideScale2Cube(boolean left)
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
        platTarget = 35;
        elevatorTarget = 35;
        forward1 = new DriveTarget(240, 0, 3, 10);
        turn1 = new GyroTurn(this.left * 45, 3, 3, 0.4);
        forward2 = new DriveTarget(8, this.left * 45, 3, 3);
        back1 = new DriveTarget(-90, this.left*45, 3, 8);
        
        arc1 = new Ellipse(20, 40, -this.left, 100, this.left * 45, 3, 5);
        arc2 = new Ellipse(20, 40, this.left, -100, 180, 3, 5);
        
        forward2 = new DriveTarget(70, this.left * 45, 3, 3);
        
        turn3 = new GyroTurn(this.left*180, 3, 3, 0.4);
        
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
                    Robot.grabber.move(-1);
                    autonFinished = true;
                   //state = autonState.wait1;
                }
                break;
            case wait1:
                moveLift();
                if (waitReps>30) {
                    reset();
                    Robot.grabber.move(1);
                    state = autonState.back1;
                    platTarget = 0;
                    elevatorTarget = 0;
                    waitReps = 0;
                }
                else {
                    waitReps++;
                }
                break;
            case back1:
                moveLift();
                if (back1.execute())
                {
                    reset();
                    Robot.grabber.move(-1);
                    state = autonState.arc1;
                }
                break;
            case arc1:
                moveLift();
                if (arc1.execute())
                {
                    reset();
                    state = autonState.arc2;
                    Robot.grabber.move(0);
                    String gameData = DriverStation.getInstance().getGameSpecificMessage().toUpperCase();
                    if ((this.left == 1 && gameData.charAt(0) == 'L') || (this.left == -1 && gameData.charAt(0) == 'R')) {
                    	state = autonState.raisePlat;
                    }
                    else {
                    	state = autonState.arc2;
                    }
                }
                break;
            case arc2:
                moveLift();
                if (arc2.execute())
                {
                    reset();
                    state = autonState.arc2;
                }
                break;
            case forward3:
                moveLift();
                if (forward3.execute())
                {
                    reset();
                    Robot.grabber.move(1);
                    autonFinished = true;
                }
                break;
            case raisePlat:
            	platTarget = 35;
            	moveLift();
            	turn3.execute();
            	if (Robot.lift.getPlatEncoder() > 30) {
            		autonFinished = true;
            		Robot.grabber.move(1);
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
        Robot.drive.AutoDrive(0, 0);
        Robot.drive.resetLeftEncoder();
        Robot.drive.resetRightEncoder();
    }
}
