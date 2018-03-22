
package org.usfirst.frc157.FRC2018.commands;

import org.usfirst.frc157.FRC2018.PID;
import org.usfirst.frc157.FRC2018.Robot;
import org.usfirst.frc157.FRC2018.commands.CenterSwitchAuton.autonState;

import edu.wpi.first.wpilibj.command.Command;

public class ArcingCenterSwitch extends Command
{
    //middle auton
    public enum autonState
    {
        arc1, arc2, turn1, arc3, turn2, forward1, back1, turn3, arc4, turn4;
    }

    private autonState state;
    private boolean autonFinished = false;
    private int left = 1;
    private double platPower;
    private double platTarget;
    private PID platPID;
    private Ellipse arc1;
    private Ellipse arc2;
    private int target1;
    private int target2;
    private GyroTurn turn1;
    private Ellipse arc3;
    private GyroTurn turn2;
    private DriveTarget forward1;
    private DriveTarget back1;
    private GyroTurn turn3;
    private Ellipse arc4;
    private GyroTurn turn4;
    private double encoder;
   

    public ArcingCenterSwitch(boolean left)
    {
        //middle auton
        requires(Robot.drive);
        state = autonState.arc1;
        platPID = new PID(1, 0, 0, 999999, 999999, 9999999, 99999);
        System.out.println("Arcing Middle Switch got instantiated");
        this.left = (left)? 1: -1;
        platTarget = 30;
        
        target1 = (int)(Math.PI*Math.sqrt((Math.pow(44, 2)+Math.pow(((this.left == 1)? 24: 34)/2, 2))/2)/2);
        arc1 = new Ellipse(44, ((this.left == 1)? 24: 34)/2, this.left, 2*target1, 0, 3, 5);
        target2 = (int)(Math.PI*Math.sqrt((Math.pow(62, 2)+Math.pow(((this.left == 1)? 24: 34)/2, 2))/2)/2);
        arc2 = new Ellipse(((this.left == 1)? 24: 34)/2,  62, -this.left, target2, (this.left*-90), 3, 5, false);
        
        turn1 = new GyroTurn(0, 2, 2, 0.4);
        int target3 = -(int)(Math.PI*Math.sqrt((Math.pow(60, 2)+Math.pow(91, 2))/2)/2);
        arc3 = new Ellipse(400, 91, this.left, target3, 0, 3, 5);
        turn2 = new GyroTurn(0, 2, 1, 0.4);
        forward1 = new DriveTarget(40, 0, 3, 3);
        back1 = new DriveTarget(-40, 0, 3, 3);
        turn3 = new GyroTurn(this.left*-90, 2, 1, 0.4);
        int target4 = (int)(Math.PI*Math.sqrt((Math.pow(65, 2)+Math.pow(40, 2))/2)/2);
        arc4 = new Ellipse(34, 120, -this.left, target4, this.left*-90, 3, 5);
        turn4 = new GyroTurn(0, 2, 2, 0.4);
    }

    @Override
    protected void execute()
    {
        switch (state)
        {
            case arc1:
            	lift();
                
                encoder = (Robot.drive.getRightEncoder() + Robot.drive.getLeftEncoder()) / 2.0;
                
                if (arc1.execute() || encoder>target2) {
                    state = autonState.arc2;
                    System.out.println("switching to arc2");
                    reset();
                }
                break;
            case arc2:
            	lift();                           
                Robot.lift.movePlat(platPower);
                 if (arc2.execute()) {
                	 Robot.grabber.move(1);
                     state = autonState.turn1;
                     reset();
                 }
                break;
            case turn1:
            	lift();
                if (turn1.execute()) {
                	state = autonState.arc3;
                    reset();
                }
                break;
            case arc3:
            	lift();                           
                Robot.lift.movePlat(platPower);
                 if (arc3.execute()) {
                	 Robot.grabber.move(-1);
                	 platTarget = 0;
                     state = autonState.turn2;
                     reset();
                 }
                break;
            case turn2:
            	lift();                           
                Robot.lift.movePlat(platPower);
                 if (turn2.execute()) {
                     state = autonState.forward1;
                     reset();
                 }
                break;
            case forward1:
            	lift();                           
                Robot.lift.movePlat(platPower);
                 if (forward1.execute()) {
                	 Robot.grabber.move(-0.5);
                     state = autonState.back1;
                     reset();
                 }
                break;
            case back1:
            	lift();                           
                Robot.lift.movePlat(platPower);
                 if (back1.execute()) {
                	 platTarget = 35;
                	 Robot.grabber.move(0);
                     state = autonState.turn3;
                     reset();
                 }
                break;
            case turn3:
            	lift();                           
                Robot.lift.movePlat(platPower);
                 if (turn3.execute()) {
                     state = autonState.arc4;
                     reset();
                 }
                break;
            case arc4:
            	lift();                           
                Robot.lift.movePlat(platPower);
                 if (arc4.execute()) {
                	 Robot.grabber.move(1);
                     state = autonState.turn4;
                     reset();
                 }
                break;
            case turn4:
            	lift();                           
                Robot.lift.movePlat(platPower);
                 if (turn4.execute()) {
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
    public void reset() {
         Robot.drive.AutoDrive(0,0);
         Robot.drive.resetLeftEncoder();
         Robot.drive.resetRightEncoder();
    }
}
