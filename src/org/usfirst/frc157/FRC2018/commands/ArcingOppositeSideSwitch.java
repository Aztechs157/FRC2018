
package org.usfirst.frc157.FRC2018.commands;

import org.usfirst.frc157.FRC2018.PID;
import org.usfirst.frc157.FRC2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ArcingOppositeSideSwitch extends Command
{
    // opposite side switch
    public enum autonState
    {
        forward1, arc1, forward2, arc2, forward3, arc3, forward4, wait1, back1;
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

    public ArcingOppositeSideSwitch(boolean left)
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
        forward1 = new DriveTarget(200, 0, 3, 5); // 144 - 72
        arc1 = new Ellipse(50,50, this.left*-1, 200, 0, 3, 2, false);
        forward2 = new DriveTarget(280, this.left * 90, 3, 5, false);
        arc2 = new Ellipse(33,33, this.left*-1, 200, this.left*90, 3, 2, false);
        forward3 = new DriveTarget(100, this.left*180, 3, 2, false);
        arc3 = new Ellipse(33, 33, this.left*-1, 200,  this.left*180, 3, 2, false);
        forward4 = new DriveTarget(22, this.left*270, 3, 2, false);
        back1 = new DriveTarget(-22, this.left*270, 3, 4);
    }

    @Override
    protected void execute()
    {
        switch (state)
        {
            case forward1:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                encoder = (Robot.drive.getRightEncoder() + Robot.drive.getLeftEncoder()) / 2.0;
                if (forward1.execute() || encoder > 104) {
                    reset();
                    state = autonState.arc1;
 
                }
                break;
            case arc1:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                encoder = (Robot.drive.getRightEncoder() + Robot.drive.getLeftEncoder()) / 2.0;
                if (arc1.execute() || encoder > 78) {
                    reset();
                    state = autonState.forward2;


                }
                break;
            case forward2:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                encoder = (Robot.drive.getRightEncoder() + Robot.drive.getLeftEncoder()) / 2.0;
                if (forward2.execute() || encoder > 132) {
                    reset();
                    state = autonState.arc2;


                }
                break;
            case arc2:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                encoder = (Robot.drive.getRightEncoder() + Robot.drive.getLeftEncoder()) / 2.0;
                if (arc2.execute() || encoder > 102) {
                    reset();
                    state = autonState.forward4;
                }
                break;
            case forward3:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                encoder = (Robot.drive.getRightEncoder() + Robot.drive.getLeftEncoder()) / 2.0;
                if (forward3.execute() || encoder > 12) {
                    reset();
                    state = autonState.arc3;
                }
                break;
            case arc3:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                encoder = (Robot.drive.getRightEncoder() + Robot.drive.getLeftEncoder()) / 2.0;
                if (arc3.execute() || encoder > 51) {
                    reset();
                    state = autonState.forward4;
                }
                break;
            case forward4:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                if (forward4.execute()) {
                    reset();
                    Robot.grabber.move(-1);
                    state = autonState.wait1;
                }
                break;
            case wait1:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                if (waitReps>50) {
                    reset();
                    Robot.grabber.move(-1);
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
       // Robot.drive.AutoDrive(0, 0);
        Robot.drive.resetLeftEncoder();
        Robot.drive.resetRightEncoder();
    }
}
