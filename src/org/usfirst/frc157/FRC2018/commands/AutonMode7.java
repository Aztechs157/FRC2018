
package org.usfirst.frc157.FRC2018.commands;

import org.usfirst.frc157.FRC2018.PID;
import org.usfirst.frc157.FRC2018.Robot;
import org.usfirst.frc157.FRC2018.SlewRate;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class AutonMode7 extends Command
{
	//same side switch
    public enum autonState 
    {
        driveArc1, wait1000Msec1, driveBack3, wait1000Msec2, turnRight90, driveArc2, notMoving;
    }

    private double startTime;
    private autonState state;
    private PID drivePID;
    private PID gyroDrivePID;
    private PID gyroPID;
    private double drivePower = 0;
    private double leftPower = 0;
    private double rightPower = 0;
    private double initAngle = 0;
    private int repsAtTarget = 0;
    private int quadrant = 0;
    private double a = 0;
    private double b = 0;
    private double c = 0;
    private double x = 0;
    private double y = 0;
    private double angle = 0;
    private double encoder;
    private int target;
    private boolean pathOpen = false;
    private boolean autonFinished = false;
    private int ellipseX;
    private int ellipseY;
    private SlewRate slewRate;
    private boolean slewCut = false;
    private int left = 1;
    private double platPower;
    private double elevatorPower;
    private double platTarget;
    private double elevatorTarget;
    private PID elevatorPID;
    private PID platPID;
    
    public AutonMode7(boolean left)
    {
    	//same side switch
        requires(Robot.drive);
        startTime = Timer.getFPGATimestamp();
        state = autonState.driveArc1;
        drivePID = new PID(0.035, 0.1, 0.000005, 10, 10, 999999, 9999999);
        gyroDrivePID = new PID(0.01, 0, 0.000001, 999999, 99999, 999999, 9999999);
        gyroPID = new PID(0.04, 0, 0.000003, 9999999, 9999999, 9999999, 999999);
        platPID = new PID(2, 0, 0, 999999, 999999, 9999999, 99999);
        System.out.println("I got called"); 
        slewRate = new SlewRate(0.001);
        this.left = (left)? 1: -1;
        platTarget = 39;
    }
    @Override
    protected void execute()
    {
    	/*if (pathOpen) {
    		pathManager.update(-(Robot.drive.getRightEncoder()+Robot.drive.getLeftEncoder())/2.0, Robot.drive.getAngle());
    	}*/
    	switch (state)
        {
    	case notMoving:
    		Robot.grabber.move(1);
        case driveArc1:
        	System.out.println("Plat Target: "+platTarget);
        	System.out.println("Plat Encoder: "+ Robot.lift.getPlatEncoder());
        	platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
        	Robot.lift.setPlatMotor(platPower);
            encoder = (Robot.drive.getRightEncoder()+Robot.drive.getLeftEncoder())/2.0;
            target = 130;
            drivePower = drivePID.pidCalculate(target, encoder);
            if(!slewCut) {
            	drivePower = slewRate.rateCalculate(drivePower);
            }
            if(Math.abs(drivePower)>=0.9) {
            	slewCut = true;
            }
            System.out.println("Right Encoder: "+Robot.drive.getRightEncoder()+"\nLeft Encoder: "+Robot.drive.getLeftEncoder());
            System.out.println("\nEncoder: " + encoder + "\nGyro: " + Robot.drive.getAngle() + "\nAngle: " + angle);

            leftPower = drivePower - gyroDrivePID.pidCalculate(initAngle, Robot.drive.getAngle());
            leftPower = ((leftPower > 0) ? 1 : -1) * Math.min(1, Math.abs(leftPower));

            rightPower = drivePower + gyroDrivePID.pidCalculate(initAngle, Robot.drive.getAngle());
            rightPower = ((rightPower > 0) ? 1 : -1) * Math.min(1, Math.abs(rightPower));

            Robot.drive.AutoDrive(leftPower, rightPower);
            if (Math.abs(encoder - target) < 5.0)
            {
                repsAtTarget++;
                if (repsAtTarget >= 5)
                {
                	reset();
                    state = autonState.wait1000Msec1;
                    System.out.println("broken 1");
                }
            }
            else
            {
                repsAtTarget = 0;
            }
            break;
        case driveArc2:
        	platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
        	Robot.lift.setPlatMotor(platPower);
            double encoder = (Robot.drive.getRightEncoder()+Robot.drive.getLeftEncoder())/2.0;
            target = 25;
            drivePower = drivePID.pidCalculate(target, encoder);
           /* if(!slewCut) {
            	drivePower = slewRate.rateCalculate(drivePower);
            }
            if(Math.abs(drivePower)>=0.9) {
            	slewCut = true;
            }*/
           /* ellipseX = 20;
            ellipseY = 120;
            x = xEllipseCalculate(ellipseX, ellipseY, encoder);
            y = yEllipseCalculate(ellipseX, ellipseY, x);
            angle = -angleEllipseCalculate(ellipseX, ellipseY, x);*/
            /*x = xSinCalculate(48,1/48.0, encoder);
            y = ySinCalculate(48,1/48.0, x);
            angle = angleSinCalculate(48,1/48.0, x);*/
            System.out.println("Right Encoder: "+Robot.drive.getRightEncoder()+"\nLeft Encoder: "+Robot.drive.getLeftEncoder());
            System.out.println("\nEncoder: " + encoder + "\nGyro: " + Robot.drive.getAngle() + "\nAngle: " + angle);

            leftPower = drivePower - gyroDrivePID.pidCalculate(initAngle, Robot.drive.getAngle());
            leftPower = ((leftPower > 0) ? 1 : -1) * Math.min(1, Math.abs(leftPower));

            rightPower = drivePower + gyroDrivePID.pidCalculate(initAngle, Robot.drive.getAngle());
            rightPower = ((rightPower > 0) ? 1 : -1) * Math.min(1, Math.abs(rightPower));

            Robot.drive.AutoDrive(leftPower, rightPower);
            if (Math.abs(encoder - target) < 3.0)
            {
                repsAtTarget++;
                if (repsAtTarget >= 2)
                {
                	Robot.grabber.move(-1);
                	autonFinished = true;
                    repsAtTarget = 0;
                }
            }
            else
            {
                repsAtTarget = 0;
            }
            break;
        case turnRight90:
        	platPower = platPID.pidCalculate(platTarget, Robot.lift.getPlatEncoder());
        	Robot.lift.setPlatMotor(platPower);
            drivePower = gyroPID.pidCalculate(left*90, Robot.drive.getAngle())*0.5;
            System.out.println("Angle: " + Robot.drive.getAngle() + "\nPower: " + drivePower);
            Robot.drive.AutoDrive(-drivePower, drivePower);
            if (Math.abs(Robot.drive.getAngle() - left*90) < 2.0)
            {
                repsAtTarget++;
                if (repsAtTarget >= 10)
                {	
                	reset();
                	slewRate = new SlewRate(0.002);
                    state = autonState.driveArc2;
                }
            }
            else
            {
                repsAtTarget = 0;
            }
            break;


        case driveBack3:
    		System.out.println("driveBack3 called");
        	encoder = (Robot.drive.getRightEncoder()+Robot.drive.getLeftEncoder())/2.0;
            target = -35;
            drivePower = drivePID.pidCalculate(target, encoder);
            
            System.out.println("Right Encoder: "+Robot.drive.getRightEncoder()+"\nLeft Encoder: "+Robot.drive.getLeftEncoder());
            System.out.println("\nEncoder: " + encoder + "\nGyro: " + Robot.drive.getAngle() + "\nAngle: " + angle);

            leftPower = drivePower - gyroDrivePID.pidCalculate(initAngle, Robot.drive.getAngle());
            leftPower = ((leftPower > 0) ? 1 : -1) * Math.min(1, Math.abs(leftPower));

            rightPower = drivePower + gyroDrivePID.pidCalculate(initAngle, Robot.drive.getAngle());
            rightPower = ((rightPower > 0) ? 1 : -1) * Math.min(1, Math.abs(rightPower));

            Robot.drive.AutoDrive(leftPower, rightPower);
            if (Math.abs(encoder - target) < 3.0)
            {
                repsAtTarget++;
                if (repsAtTarget >= 5)
                {
                	reset();
                	state = autonState.wait1000Msec2;
            		System.out.println("moving to wait1000Msec2");
                }
            }
            else
            {
                repsAtTarget = 0;
            }
            break;
        case wait1000Msec1:
        	System.out.println("wait1000Msec1");
        	repsAtTarget++;
        	System.out.println(repsAtTarget);
        	if(repsAtTarget>=50) {
        		state = autonState.turnRight90;
        		System.out.println("moving to driveBack3");
        		repsAtTarget = 0;
        	}
        	break;
        case wait1000Msec2:
    		System.out.println("wait1000Msec2 called");
        	repsAtTarget++;
        	System.out.println(repsAtTarget);
        	if(repsAtTarget>=50) {
        		state = autonState.turnRight90;
        		System.out.println("moving to turnRight90");
        		repsAtTarget = 0;
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
         repsAtTarget = 0;
         initAngle = Robot.drive.getAngle();
    }
}
