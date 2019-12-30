package frc.robot.commands;

import frc.robot.PID;
import frc.robot.Robot;
import frc.robot.SlewRate;

import edu.wpi.first.wpilibj.Timer;

public class GyroTurn
{
    private int target;
    private double startTime;
    private double time;
    private double drivePower;
    private PID gyroPID;
    private SlewRate slewRate;
    private boolean slewCut;
    private int repsAtTarget;
    private int tolerance;
    private double scalar;
    private boolean firstIteration;
    private int direction;

    public GyroTurn(int target, int tolerance, double time, double scalar)
    {
        this.target = target;
        this.time = time;
        this.tolerance = tolerance;
        this.scalar = scalar;
        slewCut = false;
        gyroPID = new PID(0.04, 0, 0.000003, 9999999, 9999999, 9999999, 999999);
        slewRate = new SlewRate(0.8);
        firstIteration = true;
        repsAtTarget = 0;
        direction = 1;
    }

    public GyroTurn(int target, int tolerance, double time, double scalar, boolean slew)
    {
        this.target = target;
        this.time = time;
        this.slewCut = slew;
        this.tolerance = tolerance;
        this.scalar = scalar;
        this.slewCut = slew;
        slewRate = new SlewRate(0.8);
        gyroPID = new PID(0.04, 0, 0.000003, 9999999, 9999999, 9999999, 999999);
        firstIteration = true;
        repsAtTarget = 0;
        direction = 1;
    }

    public boolean execute() {

        if (firstIteration) {
            startTime = Timer.getFPGATimestamp();
            slewRate.reinit();
            firstIteration = false;
           // direction = ((Math.abs(Robot.drive.getAngle()-target)>180)? -1: 1);
        }

        drivePower = direction*gyroPID.pidCalculate(target, Robot.drive.getAngle())*scalar;

        if(!slewCut) {
            drivePower = slewRate.rateCalculate(drivePower);
        }
        if(Math.abs(drivePower)>=0.9*scalar) {
            slewCut = true;
        }

        Robot.drive.AutoDrive(-drivePower, drivePower);

        if (Math.abs(Robot.drive.getAngle() - target) < tolerance)
        {
            repsAtTarget++;
            if (repsAtTarget >= 5)
            {
                return true;
            }
            else {
                return false;
            }
        }
        else if (Timer.getFPGATimestamp()-startTime>= time) {
            return true;
        }
        else
        {
            repsAtTarget = 0;
            return false;
        }
    }
}
