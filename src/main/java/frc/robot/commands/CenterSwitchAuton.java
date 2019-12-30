
package frc.robot.commands;

import frc.robot.PID;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class CenterSwitchAuton extends Command
{
	//middle auton
    public enum autonState
    {
        forward1, turn1, forward2, turn2, forward3, back1, turn3, forward4, turn4, forward5, back2, turn5, forward6, turn6, forward7
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
    private DriveTarget back1;
    private GyroTurn turn3;
    private DriveTarget forward4;
    private GyroTurn turn4;
    private DriveTarget forward5;
    private DriveTarget back2;
    private GyroTurn turn5;
    private DriveTarget forward6;
    private GyroTurn turn6;
    private DriveTarget forward7;

    public CenterSwitchAuton(boolean left)
    {
        //middle auton
        requires(Robot.drive);
        requires(Robot.grabber);
        requires(Robot.lift);
        state = autonState.forward1;
        platPID = new PID(1, 0, 0, 999999, 999999, 9999999, 99999);
        System.out.println("Middle Switch got instantiated");
        this.left = (left)? 1: -1;
        platTarget = 30;

        forward1 = new DriveTarget(44, 0, 3, 3);
        turn1 = new GyroTurn(this.left*-90, 3, 3, 0.4);
        forward2 = new DriveTarget(((this.left == 1)? 54: 44), this.left*-90, 3, 3);
        turn2 = new GyroTurn(0, 3, 3, 0.4);
        forward3 = new DriveTarget(58, 0, 3, 3);
        back1 = new DriveTarget(-51, 0, 3, 3);
        turn3 = new GyroTurn(this.left*90, 3, 3, 0.4);
        forward4 = new DriveTarget(((this.left == 1)? 54: 44), this.left*90, 3, 3);
        turn4 = turn2 = new GyroTurn(0, 3, 3, 0.4);
        forward5 = new DriveTarget(30, 0, 3, 3);
        back2 = new DriveTarget(-30, 0, 3, 3);
        turn5 = new GyroTurn(this.left*-90, 3, 3, 0.4);
        forward6 = new DriveTarget(((this.left == 1)? 54: 44), this.left*-90, 3, 3);
        turn6 = new GyroTurn(0, 3, 3, 0.4);
        forward7 = new DriveTarget(51, 0, 3, 3);
    }

    @Override
    protected void execute()
    {
        switch (state)
        {
            case forward1:
                lift();
                if (forward1.execute()) {
                    state = autonState.turn1;
                    reset();
                }
                break;
            case turn1:
                lift();
                if (turn1.execute()) {
                    state = autonState.forward2;
                    reset();
                }
                break;
            case forward2:
                lift();
                if (forward2.execute()) {
                     state = autonState.turn2;
                     reset();
                 }
                break;
            case turn2:
                lift();
                if (turn2.execute()) {
                    state = autonState.forward3;
                    reset();
                }
                break;
            case forward3:
                lift();
                if (forward3.execute()) {
                    Robot.grabber.move(-1);
                    state = autonState.back1;
                   //autonFinished = true;
                    reset();
                }
                break;
            case back1:
                lift();
                if (back1.execute()) {
                    //Robot.grabber.move(-1);
                    state = autonState.turn3;
                    autonFinished = true;
                   // platTarget = 0;
                    reset();
                }
                break;
            case turn3:
                lift();
                if (turn3.execute()) {
                    state = autonState.forward4;
                    reset();
                }
                break;
            case forward4:
                lift();
                if (forward4.execute()) {
                    state = autonState.turn4;
                    reset();
                }
                break;
            case turn4:
                lift();
                if (turn4.execute()) {
                    state = autonState.forward5;
                    reset();
                }
                break;
            case forward5:
                lift();
                if (forward5.execute()) {
                    state = autonState.back2;
                    Robot.grabber.move(0);
                    reset();
                }
                break;
            case back2:
                lift();
                if (back2.execute()) {
                    state = autonState.turn5;
                    platTarget = 35;
                    reset();
                }
                break;
            case turn5:
                lift();
                if (turn5.execute()) {
                    state = autonState.forward6;
                    reset();
                }
                break;
            case forward6:
                lift();
                if (forward6.execute()) {
                    state = autonState.turn6;
                    reset();
                }
                break;
            case turn6:
                lift();
                if (turn6.execute()) {
                    state = autonState.forward7;
                    reset();
                }
                break;
            case forward7:
                lift();
                if (forward7.execute()) {
                    Robot.grabber.move(1);
                    autonFinished = true;
                    reset();
                }
                break;
        }
    }
    public void lift() {
        platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
        Robot.lift.movePlat(platPower);
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

    public void reset() {
        Robot.drive.AutoDrive(0,0);
        Robot.drive.resetLeftEncoder();
        Robot.drive.resetRightEncoder();
    }
}
