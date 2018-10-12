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

import org.usfirst.frc157.FRC2018.OI;

//import java.text.DecimalFormat ; 

import org.usfirst.frc157.FRC2018.Robot;
import org.usfirst.frc157.FRC2018.RobotMap;
import org.usfirst.frc157.FRC2018.SlewRate;
import org.usfirst.frc157.FRC2018.commands.DriveWithSticks;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.AnalogGyro;
//import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
//import edu.wpi.first.wpilibj.interfaces.Accelerometer;
//import org.usfirst.frc157.FRC2018.OI;


/**
 *
 */
public class Drive extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
    private static int count=0;
    public boolean onestick = false;
    public boolean AttackDrive = false;
    public boolean ContrDrive = false;
    public boolean mechDrive = true;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final Encoder leftQuad = RobotMap.drivedriveLeftQuad;
    private final Encoder rightQuad = RobotMap.driveRightQuad;
    private final WPI_TalonSRX driveLeft1 = RobotMap.driveLeft1;
    private final WPI_TalonSRX driveLeft2 = RobotMap.driveLeft2;
    private final WPI_TalonSRX driveRight1 = RobotMap.driveRight1;
    private final WPI_TalonSRX driveRight2 = RobotMap.driveRight2;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    //private final Accelerometer accel = RobotMap.accel;
    private final AnalogGyro gyro = RobotMap.driveGyro;
    private SlewRate leftSlew;
    private SlewRate rightSlew;
    //private final AnalogPotentiometer analogPotentiometer = RobotMap.motorControllerTeAnalogPotentiometer;
    
    public Drive() {
        leftSlew = new SlewRate(1.2);
        rightSlew = new SlewRate(1.2);
    }
    @Override
    public void initDefaultCommand()
    {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
        driveRight1.set(ControlMode.PercentOutput, 0.0);
        driveRight2.set(ControlMode.Follower,  driveRight1.getDeviceID());
        driveLeft1.set(ControlMode.PercentOutput, 0.0);
        driveLeft2.set(ControlMode.Follower,  driveLeft1.getDeviceID());
        System.out.println("base id: " + driveLeft1.getBaseID()+ "\ndevice id: " + driveLeft1.getDeviceID());
        /*driveRight1.setNeutralMode(NeutralMode.Brake);
        driveRight2.setNeutralMode(NeutralMode.Brake);
        driveLeft1.setNeutralMode(NeutralMode.Brake);
        driveLeft2.setNeutralMode(NeutralMode.Brake);*/
        setDefaultCommand(new DriveWithSticks());
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    public double getAngle()
    {
        double angle = -gyro.getAngle();
//        if (Math.abs(angle)>180) {
//            angle = ((angle>0)? -1: 1)*(360-Math.abs(angle));
//        }
        return angle;
    }
    public void DriveRobot()
    {
        //DecimalFormat numberFormat = new DecimalFormat("0.00");
        double potentiometer = RobotMap.speedScale.get();
        double left = Robot.oi.getgamePad().getRawAxis(OI.RYStick)*potentiometer;
        double right = -Robot.oi.getgamePad().getRawAxis(OI.LYStick)*potentiometer;
        left = /*((left>=0)? 1: -1)*/Math.pow(left,3)*0.80;
        right = /*((right>=0)? 1: -1)*/Math.pow(right,3)*0.80;
        if (onestick==true)/////switched this to get mechanum only probably wrong but its 1:07am and i dont care
        {
            driveRightSet(left); // Change to RDrive * potentiometer
            driveLeftSet(right); // Change to LDrive * potentiometer
        }
        else
        {
          //   mechDrive = false;
            if (mechDrive)
            {
                double RxDrive = -Robot.oi.getgamePad().getRawAxis(OI.LYStick);
                double lYDrive = Robot.oi.getgamePad().getRawAxis(OI.RxStick);
                driveRightSet(lYDrive-RxDrive);
                driveLeftSet(lYDrive+RxDrive);
               // driveSet(lYDrive+RxDrive,lYDrive-RxDrive );
             
            }
            else
            {
             //   double RxDrive = Robot.oi.getgamePad().getRawAxis(OI.RxStick) * potentiometer;
                double RxDrive = -Robot.oi.getgamePad().getRawAxis(OI.RYStick);
                double lYDrive = Robot.oi.getgamePad().getRawAxis(OI.RxStick);
                driveRightSet(lYDrive-RxDrive);
                driveLeftSet(lYDrive+RxDrive);
                {
                     System.out.println("tank drive");
                }
            }
        }
        
        
    }
    public void driveRightSet(double power) {
        power = rightSlew.rateCalculate(power);
        driveRight1.set(power);
    }
    public void driveLeftSet(double power) {
        power = leftSlew.rateCalculate(power);
        driveLeft1.set(power);
    }
    public void driveSet(double leftPower, double rightPower) {
        if ((leftPower > 0 && rightPower>0) || (leftPower<0 && rightPower<0)) {
            leftPower = leftSlew.rateCalculate(leftPower);
            rightPower = rightSlew.rateCalculate(rightPower);
        }
        else {
            leftPower = leftSlew.rateCalculate(leftPower, 1.8);
            rightPower = rightSlew.rateCalculate(rightPower, 1.8);
        }
        driveLeft1.set(leftPower);
        driveRight1.set(rightPower);
    }
    public double getLeftEncoder()
    {
        return leftQuad.getDistance();
    }
    public void resetLeftEncoder()
    {
        leftQuad.reset();
    }
    public double getRightEncoder()
    {
        return rightQuad.getDistance();
    }
    public void resetRightEncoder()
    {
        rightQuad.reset();
    }
    public void AutoDrive(double Rdrive, double Ldrive)
    {
        driveRight1.set(-Rdrive);
        driveLeft1.set(Ldrive);
    }

    /*public double[] getAccelerometer()
    {
        return new double[]{accel.getX(), accel.getY(), accel.getZ()};
    }*/
    public String debugPrint()
    {
        return "Right Encoder: " + getRightEncoder() + 
                "\nLeft Encoder" + getLeftEncoder() + 
                "\ngyro: " + getAngle();
    }
    public void brake()
    {
    }
    @Override
    public void periodic()
    {
        // Put code here to be run every loop
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}