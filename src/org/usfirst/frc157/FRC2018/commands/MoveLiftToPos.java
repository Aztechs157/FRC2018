// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc157.FRC2018.commands;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc157.FRC2018.PID;
import org.usfirst.frc157.FRC2018.Robot;
import org.usfirst.frc157.FRC2018.subsystems.Lift;

/**
 *
 */
public class MoveLiftToPos extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    private int m_pos;
    private static final double startPos = 3;
    private static final double carriage1 = 440;
    private static final double top = 80;
    private static final double switchHeight = 18.75-startPos;
    private static final double scaleLow = (4*12)-startPos;
    private static final double scaleMed = (5*12)-startPos;
    private static final double scaleHigh = (60*12)-startPos;
    private static final double portalHeight = (12+8)-startPos;
    private static final double bottom = 0;
    private static final double stops[] = new double[] {bottom, switchHeight, portalHeight, scaleLow, scaleMed, scaleHigh, top};
    public double posPlatFinal;
    private double posStageFinal;
    private double distPlat;
    private double distStage;
    private double platTime;
    private double stageTime;
    private double posPlatInterm;
    private boolean platFirst;
    private double platSpeed = 4.5;// 20.64;
    private double stageSpeed = 5;//28.2;
    private Lift.quad encoder = Lift.quad.PLATFORM;
    private Lift.direction direction = Lift.direction.UP;
    private PID platPID = new PID(0.05, 0, 0, 999999, 99999, 999999, 9999999);
    private PID stagePID = new PID(0.05, 0, 0, 999999, 99999, 999999, 9999999);
    double destPos;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public MoveLiftToPos(int pos) {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        m_pos = pos;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.lift);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        destPos = stops[m_pos];
        /*encoder = Lift.quad.PLATFORM;
        direction = Lift.direction.UP;
        if (destPos > carriage1)
        {
            encoder = Lift.quad.EXTENSION;
            destPos -= 40;
        }
        if (Robot.lift.getEncoder(encoder) > destPos)
        {
            direction = Lift.direction.DOWN;
        }
        */
        
        stageSpeed = SmartDashboard.getNumber("Stage Speed", stageSpeed);
        SmartDashboard.putNumber("stageSpeedVal", stageSpeed);
        
        double[] distances = getDistances(destPos);
        posPlatFinal = distances[0];
        posStageFinal = distances[1];
        distPlat = posPlatFinal - Robot.lift.getPlatEncoder();
        distStage = posStageFinal - Robot.lift.getStageEncoder();
        platTime = Math.abs(distPlat/platSpeed);
        stageTime = Math.abs(distStage/stageSpeed);
        System.out.println(platTime);
        System.out.println(stageTime);
        if (platTime > stageTime && destPos > Robot.lift.getPlatEncoder() + Robot.lift.getStageEncoder())
        {
            posPlatInterm = getPpi();
            platFirst = true;
        }
        else
        {
            platFirst = false;
        }
        SmartDashboard.putNumber("platFinalPos", posStageFinal);
    }
    private double getPpi()
    {
        double platTimeInterm = 0;
        if (platTime > stageTime)
        {
            platTimeInterm = platTime - stageTime;
        }
        System.out.println(platTimeInterm);
        double Ppi = Robot.lift.getPlatEncoder() + (platSpeed*platTimeInterm);
        return Ppi;
    }
    private double[] getDistances(double totalDist)
    {
        double[] retVal = {0, 0};
        if (destPos > carriage1)
        {
            retVal[0] = carriage1;
            retVal[1] = destPos - carriage1;
        }
        else
        {
            retVal[0] = destPos;
            retVal[1] = 0;
        }
        return retVal;
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        SmartDashboard.putNumber("platCurrent", Robot.lift.getPlatEncoder());
        SmartDashboard.putBoolean("moving", true);
        if (platFirst)
        {
            SmartDashboard.putNumber("Ppi", posPlatInterm);
            SmartDashboard.putBoolean("test", true);
            if (Robot.lift.getPlatEncoder() < posPlatInterm-0.5)
            {
                Robot.lift.movePlat(platPID.pidCalculate(posPlatFinal, Robot.lift.getPlatEncoder()));
            }
            else
            {
                Robot.lift.stopPlat();
                platFirst = false;
            }
        }
        else
        {

            SmartDashboard.putBoolean("test", false);
            SmartDashboard.putBoolean("test1", Robot.lift.getPlatEncoder() < posPlatFinal-.5);
            SmartDashboard.putBoolean("test2", Robot.lift.getPlatEncoder() > posPlatFinal-.5);
            if (Robot.lift.getPlatEncoder() < posPlatFinal-.5)
            {
                Robot.lift.movePlat(platPID.pidCalculate(posPlatFinal, Robot.lift.getPlatEncoder()));
            }
            else if (Robot.lift.getPlatEncoder() > posPlatFinal+.5)
            {
                Robot.lift.movePlat(platPID.pidCalculate(posPlatFinal, Robot.lift.getPlatEncoder()));
            }
            else
            {
                Robot.lift.stopPlat();
            }
            SmartDashboard.putBoolean("test3", Robot.lift.getStageEncoder() < posStageFinal - .5);
            SmartDashboard.putBoolean("test4", Robot.lift.getStageEncoder() > posStageFinal - .5);
            if (Robot.lift.getStageEncoder() < posStageFinal - .5)
            {
                Robot.lift.moveStage(stagePID.pidCalculate(posStageFinal, Robot.lift.getStageEncoder()));
            }
            else if (Robot.lift.getStageEncoder() > posPlatFinal + .5)
            {
                Robot.lift.moveStage(stagePID.pidCalculate(posStageFinal, Robot.lift.getStageEncoder()));
            }
            else
            {
                Robot.lift.stopStage();
            }
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        double pos = Robot.lift.getPlatEncoder();
        boolean finished = false;
        if (pos >= destPos - .2 && pos <= destPos + .2)
        {
            pos = Robot.lift.getStageEncoder();
            if (pos >= destPos - .2 && pos <= destPos + .2)
            {
                finished = true;
            }
        }
        return finished;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.lift.stop();

        SmartDashboard.putBoolean("moving", false);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        Robot.lift.stop();

        SmartDashboard.putBoolean("moving", false);
    }
}