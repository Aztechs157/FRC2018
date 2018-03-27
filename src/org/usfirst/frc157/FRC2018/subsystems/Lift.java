// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc157.FRC2018.subsystems;

import org.usfirst.frc157.FRC2018.PID;

//import java.text.DecimalFormat;

//import org.usfirst.frc157.FRC2018.Robot;
import org.usfirst.frc157.FRC2018.RobotMap;
import org.usfirst.frc157.FRC2018.commands.*;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class Lift extends Subsystem
{

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final DigitalInput extensionTopLimitxxx = RobotMap.stageTopLimit;
    private final DigitalInput extensionBottomLimit = RobotMap.stageBottomLimit;
    private final WPI_TalonSRX extensionMotor = RobotMap.stageTalon;
    private final DigitalInput platformTopLimit = RobotMap.platformTopLimit;
    private final DigitalInput platformBottomLimit = RobotMap.platformBottomLimit;

    private final WPI_TalonSRX platformMotor = RobotMap.platformTalon;
    private final Encoder platformQuad = RobotMap.platformQuad;
    private final Encoder extensionQuad = RobotMap.stageQuad;
    private final double scale = 1;
    public static final double STAGETOP = 35;
    public static final double PLATTOP = 40.5;
    public PID platTopPID = new PID(1, 0, 0.00000, 999999, 99999, 999999, 9999999);
    public PID platPID = new PID(0.6, 0, 0.0000000, 999999, 99999, 999999, 9999999);
    public PID platDownPID = new PID(0.5, 0, 0.00003, 999999, 99999, 999999, 9999999);
    public PID stagePID = new PID(0.5, 0, 0, 999999, 99999, 999999, 9999999);
    public PID stageDownPID = new PID(0.5, 0, 0.00001, 999999, 99999, 999999, 9999999);
    public static double platLast = 0;
    public static double stageLast = 0;
    public static enum direction
    {
        UP, DOWN,
    }

    public static enum quad
    {
        PLATFORM, STAGE
    }
    public boolean StageTopLimit()
    {
        return extensionTopLimitxxx.get();
    }
    public void hold()
    {
        platLast = getPlatEncoder();
        stageLast = getStageEncoder();

    }

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    @Override
    public void initDefaultCommand()
    {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
        platformMotor.set(ControlMode.PercentOutput, 0.0);

        extensionMotor.set(ControlMode.PercentOutput, 0.0);
        setDefaultCommand(new StopLift());
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    
    public void movePlat(double speed)
    {
        platLast = Double.NaN;
        if (speed > 0)
        {
            if (!platformTopLimit.get())
            {
                platformMotor.set(speed);
            }
            else
            {
                stopPlat();
            }
        }
        else
        {
            if (!false)
            {
                platformMotor.set(speed);
            }
            else
            {
                stopPlat();
            }
        }
    }
    public void movePlatNoReset(double speed)
    {
        //platLast = Double.NaN;
        if (speed > 0)
        {
            if (!platformTopLimit.get())
            {
                platformMotor.set(speed);
            }
            else
            {
                //System.out.println("movePlatNoReset stopped me");
                platformMotor.set(0);
            }
        }
        else
        {
            if (!false)
            {
                platformMotor.set(speed);
            }
            else
            {
                platformMotor.set(0);;
            }
        }
    }
    public void moveStageNoReset(double speed)
    {
        if (speed < 0)
        {
            if (!StageTopLimit())
            {
                extensionMotor.set(speed);
            }
            else
            {
                extensionMotor.set(0);
            }
        }
        else
        {
            if (!extensionBottomLimit.get())
            {
                extensionMotor.set(speed);
            }
            else
            {
                extensionMotor.set(0);
            }
        }
    }
    
    public void setStage(double speed)
    {
        extensionMotor.set(speed);
    }
    
    public void setPlat(double speed)
    {
        platformMotor.set(speed);
    }
    
    public void moveStage(double speed)
    {
        stageLast = Double.NaN;
        if (speed < 0)
        {
            if (!StageTopLimit())
            {
                extensionMotor.set(speed);
            }
            else
            {
                stopStage();
            }
        }
        else
        {
            //System.out.println(!extensionBottomLimit.get());
            if (!extensionBottomLimit.get())
            {
                extensionMotor.set(speed);
            }
            else
            {
                stopStage();
            }
        }
    }
    public void stopPlat()
    {
        platLast = (Double.isNaN(platLast))?getPlatEncoder():platLast;
        movePlatNoReset(scale*platPID.pidCalculate(platLast, getPlatEncoder()));
    }
    public void stopStage()
    {
        stageLast = (Double.isNaN(stageLast))?getStageEncoder():stageLast;
        moveStageNoReset(-scale*stagePID.pidCalculate(stageLast, getStageEncoder()));
    }
    public void move(direction dir, double speed)
    {
       switch (dir)
       {
           case UP:
               if (getPlatEncoder() >= PLATTOP - 0.25)
               {

                   platLast = (Double.isNaN(platLast))?getPlatEncoder():platLast;
                   moveStage(-scale*stagePID.pidCalculate(STAGETOP, getStageEncoder()));
                   stageLast = Double.NaN;
                   movePlat(scale*platTopPID.pidCalculate(PLATTOP, getPlatEncoder()));
                   //System.out.println(-scale*stagePID.pidCalculate(STAGETOP, getStageEncoder()));
               }
               else
               {
                   stageLast = (Double.isNaN(stageLast))?getStageEncoder():stageLast;
                   movePlat(scale*platPID.pidCalculate(PLATTOP, getPlatEncoder()));
                   platLast = Double.NaN;
                   //moveStage(-scale*platPID.pidCalculate(stageLast, getStageEncoder()));
                   //System.out.println(stageLast);
               }
               break;
           case DOWN:
               if (getStageEncoder() > 0.25)
               {
                   platLast = (Double.isNaN(platLast))?getPlatEncoder():platLast;
                   moveStage(-scale*stageDownPID.pidCalculate(0, getStageEncoder()));
                   stageLast = Double.NaN;
                   movePlat(scale*platPID.pidCalculate(platLast, getPlatEncoder()));
               }
               else
               {
                   stageLast = (Double.isNaN(stageLast))?getStageEncoder():stageLast;
                   movePlat(scale*platDownPID.pidCalculate(0, getPlatEncoder()));
                   platLast = Double.NaN;
                   moveStage(-scale*stagePID.pidCalculate(0, getStageEncoder()));
               }
               break;
        }
    }
    public void stop()
    {
        stageLast = (Double.isNaN(stageLast))?getStageEncoder():stageLast;
        platLast = (Double.isNaN(platLast))?getPlatEncoder():platLast;
        moveStageNoReset(-scale*stagePID.pidCalculate(stageLast, getStageEncoder()));
        movePlatNoReset(scale*platPID.pidCalculate(platLast, getPlatEncoder()));
    }
    public double getEncoder(quad encoder)
    {
        double retVal = 0;
        switch (encoder)
        {
            case PLATFORM:
                retVal = getPlatEncoder();
                break;
            case STAGE:
                retVal = getStageEncoder();
                break;
        }
        return retVal;
    }
    public double getPlatEncoder()
    {
        return platformQuad.getDistance();
    }

    public double getStageEncoder()
    {
        return extensionQuad.getDistance();
    }
    /**
     * @return the extensionTopLimit
     */
    public boolean getExtensionTopLimit()
    {
        return StageTopLimit();
    }
    /**
     * @return the extensionBottomLimit
     */
    public boolean getExtensionBottomLimit()
    {
        return extensionBottomLimit.get();
    }
    /**
     * @return the platformTopLimit
     */
    public boolean getPlatformTopLimit()
    {
        return platformTopLimit.get();
    }
    /**
     * @return the platformBottomLimit
     */
    public boolean getPlatformBottomLimit()
    {
        return false;
    }
    public String debugPrint()
    {
        return "Platform Encoder: "+getPlatEncoder() + "\nStage Encoder: "+getStageEncoder();
    }
    public void resetPlatEncoder()
    {
        platformQuad.reset();
    }
    public void resetStageEncoder()
    {
        extensionQuad.reset();
    }
    @Override
    public void periodic()
    {
        // Put code here to be run every loop
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}