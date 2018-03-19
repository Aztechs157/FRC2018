
package org.usfirst.frc157.FRC2018.commands;

import org.usfirst.frc157.FRC2018.PID;
import org.usfirst.frc157.FRC2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ArcingCenterSwitch extends Command
{
    //middle auton
    public enum autonState 
    {
        arc1, arc2;
    }

    private autonState state;
    private boolean autonFinished = false;
    private int left = 1;
    private double platPower;
    private double platTarget;
    private PID platPID;
    private Ellipse arc1;
    private Ellipse arc2;
    
    public ArcingCenterSwitch(boolean left)
    {
        //middle auton
        requires(Robot.drive);
        state = autonState.arc1;
        platPID = new PID(1, 0, 0, 999999, 999999, 9999999, 99999);
        System.out.println("Middle Switch got instantiated"); 
        this.left = (left)? 1: -1;
        platTarget = 30;
        
        arc1 = new Ellipse(44, ((this.left == 1)? 54: 44), (this.left == 1)? )
    }

    @Override
    protected void execute()
    {
        switch (state)
        {
            case arc1:
                platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
                Robot.lift.movePlat(platPower);
                if (forward1.execute()) {
                    state = autonState.turn1;
                    reset();
                }
                break;
            case arc2:
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
