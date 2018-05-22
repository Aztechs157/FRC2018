// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc157.FRC2018;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static WPI_TalonSRX platformTalon;
    public static Encoder platformQuad;
    public static DigitalInput platformTopLimit;
    public static DigitalInput platformBottomLimit;
    public static Encoder stageQuad;
    public static WPI_TalonSRX stageTalon;
    public static DigitalInput stageTopLimit;
    public static DigitalInput stageBottomLimit;
    public static WPI_TalonSRX driveRight1;
    public static WPI_TalonSRX driveRight2;
    public static WPI_TalonSRX driveLeft1;
    public static WPI_TalonSRX driveLeft2;
    public static WPI_TalonSRX climberMotor;
    public static Encoder driveRightQuad;
    public static Encoder drivedriveLeftQuad;
    public static AnalogGyro driveGyro;
    public static WPI_TalonSRX intakeRightTalon;
    public static WPI_TalonSRX intakeLeftTalon;
    public static AnalogPotentiometer autoSelect;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    @SuppressWarnings("deprecation")
    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        platformTalon = new WPI_TalonSRX(5);
        
        
        platformQuad = new Encoder(4, 5, false, EncodingType.k4X);
        LiveWindow.addSensor("Platform", "platformQuad", platformQuad);
        platformQuad.setDistancePerPulse(0.005);
        platformQuad.setDistancePerPulse(0.005);

        platformQuad.setPIDSourceType(PIDSourceType.kRate);
        platformTopLimit = new DigitalInput(6);
        LiveWindow.addSensor("Platform", "platformTopLimit", platformTopLimit);
        
        platformBottomLimit = new DigitalInput(7);
        LiveWindow.addSensor("Platform", "platformBottomLimit", platformBottomLimit);
        
        stageQuad = new Encoder(21, 22, false, EncodingType.k4X);
        LiveWindow.addSensor("Extension", "carriageQuad", stageQuad);

        stageQuad.setDistancePerPulse(0.009); 

        stageQuad.setDistancePerPulse(0.009);

        stageQuad.setPIDSourceType(PIDSourceType.kRate);
        stageTalon = new WPI_TalonSRX(7);
        
        
        stageTopLimit = new DigitalInput(23);
        LiveWindow.addSensor("Extension", "extensionTopLimit", stageTopLimit);
        
        stageBottomLimit = new DigitalInput(20);
        LiveWindow.addSensor("Extension", "extensionBottomLimit", stageBottomLimit);
        
        driveRight1 = new WPI_TalonSRX(1);
        
        
        driveRight2 = new WPI_TalonSRX(2);
        
        
        driveLeft1 = new WPI_TalonSRX(8);
        
        
        driveLeft2 = new WPI_TalonSRX(9);
        
        
        driveRightQuad = new Encoder(2, 3, false, EncodingType.k4X);
        LiveWindow.addSensor("Drive", "driveRightQuad", driveRightQuad);
        driveRightQuad.setDistancePerPulse(0.052333);
        driveRightQuad.setPIDSourceType(PIDSourceType.kRate);
        drivedriveLeftQuad = new Encoder(0, 1, true, EncodingType.k4X);
        LiveWindow.addSensor("Drive", "driveLeftQuad", drivedriveLeftQuad);
        drivedriveLeftQuad.setDistancePerPulse(0.052333);
        drivedriveLeftQuad.setPIDSourceType(PIDSourceType.kRate);
        driveGyro = new AnalogGyro(0);
        LiveWindow.addSensor("Drive", "driveGyro", driveGyro);
        driveGyro.setSensitivity(0.007);
        intakeRightTalon = new WPI_TalonSRX(3);
        
        
        intakeLeftTalon = new WPI_TalonSRX(4);
        autoSelect = new AnalogPotentiometer(2, 11.75, 0.0);

        climberMotor = new WPI_TalonSRX(6);

        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
}
