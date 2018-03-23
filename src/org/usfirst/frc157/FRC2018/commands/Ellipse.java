package org.usfirst.frc157.FRC2018.commands;

import org.usfirst.frc157.FRC2018.PID;
import org.usfirst.frc157.FRC2018.Robot;
import org.usfirst.frc157.FRC2018.SlewRate;

import edu.wpi.first.wpilibj.Timer;

public class Ellipse
{
    private int target;
    private double startTime;
    private double time;
    private double targetAngle;
    private boolean slewCut;
    private double encoder;
    private double drivePower;
    private PID drivePID;
    private PID gyroDrivePID;
    private SlewRate slewRate;
    private double leftPower;
    private double rightPower;
    private int repsAtTarget;
    private int tolerance;
    private boolean firstIteration;
    private double a;
    private double b;
    private int quadrant;
    private int direction;

    public Ellipse(double a, double b, int direction, int target, double targetAngle, int tolerance, double time)
    {
        this.a = a;
        this.b = b;
        this.direction = direction;
        this.target = target;
        this.time = time;
        this.targetAngle = targetAngle;
        this.tolerance = tolerance;
        quadrant = 1;
        slewCut = false;
        drivePID = new PID(0.028, 0.1, 0.000005, 10, 10, 999999, 9999999);
        gyroDrivePID = new PID(0.01, 0, 0.000001, 999999, 99999, 999999, 9999999);
        slewRate = new SlewRate(1.2);
        firstIteration = true;
    }

    public Ellipse(double a, double b, int direction, int target, double targetAngle, int tolerance, double time, boolean slew)
    {
    	this.a = a;
        this.b = b;
        this.direction = direction;
        this.target = target;
        this.time = time;
        this.targetAngle = targetAngle;
        this.tolerance = tolerance;
        quadrant = 1;
        slewCut = !slew;
        drivePID = new PID(0.028, 0.1, 0.000005, 10, 10, 999999, 9999999);
        gyroDrivePID = new PID(0.01, 0, 0.000001, 999999, 99999, 999999, 9999999);
        slewRate = new SlewRate(1.2);
        firstIteration = true;
    }

    public boolean execute()
    {

        if (firstIteration)
        {
            startTime = Timer.getFPGATimestamp();
            slewRate.reinit();
            firstIteration = false;
        }

        encoder = (Robot.drive.getRightEncoder() + Robot.drive.getLeftEncoder()) / 2.0;
        drivePower = drivePID.pidCalculate(target, encoder);
        if (!slewCut)
        {
            drivePower = slewRate.rateCalculate(drivePower);
        }
        if (Math.abs(drivePower) >= 0.9)
        {
            slewCut = true;
        }
        
        double x = xEllipseCalculate(encoder);
        double angle = direction*angleEllipseCalculate(x);
        
        System.out.println("X: "+ x);
        System.out.println("Angle: "+ angle);
        
        leftPower = drivePower - gyroDrivePID.pidCalculate(targetAngle + angle, Robot.drive.getAngle());
        leftPower = ((leftPower > 0) ? 1 : -1) * Math.min(1, Math.abs(leftPower));

        rightPower = drivePower + gyroDrivePID.pidCalculate(targetAngle + angle, Robot.drive.getAngle());
        rightPower = ((rightPower > 0) ? 1 : -1) * Math.min(1, Math.abs(rightPower));

        Robot.drive.AutoDrive(leftPower, rightPower);
        if (Math.abs(encoder - target) < tolerance)
        {
            repsAtTarget++;
            if (repsAtTarget >= 5)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else if (Timer.getFPGATimestamp() - startTime >= time)
        {
            return true;
        }
        else
        {
            repsAtTarget = 0;
            return false;
        }
    }

    public double xEllipseCalculate(double distance)
    {
        double semiperimeter = Math.PI * Math.sqrt((a * a + b * b) / 2) / 2.0;
        quadrant = (int) (distance / semiperimeter) + 1;
        System.out.println(quadrant);
        distance = distance - ((quadrant - 1) * semiperimeter);
        if (quadrant == 2 || quadrant == 4)
        {
            distance = semiperimeter - distance;
        }
        System.out.println(distance);
        double origA = a;
        double tempB = b / a;
        distance = distance / a;
        double tempA = 1;
        double sum = 0;
        double curX = 0;
        double calcSlice = 0;
        double deltaX = 0.05;
        while (Math.abs(sum - distance) > 0.05)
        {
            // System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
            calcSlice = Math.sqrt(1 + Math.pow(-curX / (tempA * Math.sqrt(tempA * tempA - curX * curX)), 2)) * deltaX;
            sum += (calcSlice);

            curX += deltaX;
        }
        // System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
        double returnVal = (curX > tempA) ? tempA * origA : curX * origA;
        return (quadrant == 3 || quadrant == 4) ? -returnVal : returnVal;
    }

    public double yEllipseCalculate(double x)
    {
        double origA = a;
        double tempB = b / a;
        double tempX = x / a;
        double tempA = 1;
        double y = Math.sqrt((1 * tempB * tempB - tempB * tempB * tempX * tempX) / (tempA * tempA));
        return origA * y;
    }

    public double angleEllipseCalculate(double x)
    {
    	/*
        x = Math.abs(x);
        double slope = -x / (a*Math.sqrt(a*a - x * x));
        double angle = Math.toDegrees(Math.atan(slope)); // ((quadrant==1||quadrant==3)? 1:-1)
        if (quadrant == 2)
        {
            angle = ((angle >= 0) ? 1 : -1) * (180 - Math.abs(angle));
        }
        return angle;
        */
    	double tempB = b / a;
		double tempX = x / a;
		double tempA = 1;
		double slope = -tempX/(tempA*Math.sqrt(tempA*tempA-tempX*tempX));
		double angle = Math.toDegrees(Math.atan(slope));
		if (quadrant == 2)
        {
            angle = ((angle >= 0) ? 1 : -1) * (180 - Math.abs(angle));
        }
		return angle;
    }
}
