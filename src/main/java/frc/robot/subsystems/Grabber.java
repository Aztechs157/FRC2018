// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package frc.robot.subsystems;

import frc.robot.OI;
import frc.robot.Robot;

//import java.text.DecimalFormat ; 

//import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.SlewRate;
import frc.robot.commands.GrabberWithTriggers;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import edu.wpi.first.wpilibj.CounterBase.EncodingType;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.AnalogGyro;
//import edu.wpi.first.wpilibj.AnalogPotentiometer;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.PIDSourceType;

//import frc.robot.commands.*;
import edu.wpi.first.wpilibj.command.Subsystem;
//import edu.wpi.first.wpilibj.interfaces.Accelerometer;
//import frc.robot.OI;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Grabber extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final WPI_TalonSRX intakeLeft = RobotMap.intakeLeftTalon;
    private final WPI_TalonSRX intakeRight = RobotMap.intakeRightTalon;
    private SlewRate slewRate;

    double RTAxis;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public Grabber() {
        slewRate = new SlewRate(2.5);
    }
    @Override
    public void initDefaultCommand()
    {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
        intakeRight.set(ControlMode.PercentOutput, 0.0);
        intakeLeft.set(ControlMode.Follower, intakeRight.getDeviceID());
        /*driveRight1.setNeutralMode(NeutralMode.Brake);
        driveRight2.setNeutralMode(NeutralMode.Brake);
        driveLeft1.setNeutralMode(NeutralMode.Brake);
        driveLeft2.setNeutralMode(NeutralMode.Brake);*/
        //Set the default command for a subsystem here.
        setDefaultCommand(new GrabberWithTriggers());
    }
    public String debugPrint()
    {
        return "";
    }
    public void move(double speed)
    {
        //speed = slewRate.rateCalculate(speed);
        intakeLeft.set(speed);
        intakeRight.set(-speed);
    }
    @Override
    public void periodic()
    {
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

