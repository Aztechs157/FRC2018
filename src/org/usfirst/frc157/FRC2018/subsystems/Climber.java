// Code that implements a camera into the driver sation
package org.usfirst.frc157.FRC2018.subsystems;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc157.FRC2018.RobotMap;
import org.usfirst.frc157.FRC2018.commands.ClimberController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//import org.usfirst.frc157.FRC2018.commands.CameraController;
//import org.usfirst.frc157.FRC2018.commands.GrabberWithTriggers;

/**
 *  @AUTH Tyler Silva.
 */
public class Climber extends Subsystem {
    public boolean debugMode = true; // Change to true to view camera stats
    /**
     * 
     */
    private WPI_TalonSRX motor = RobotMap.climberMotor;
    @Override
    public void initDefaultCommand()
    {
        setDefaultCommand(new ClimberController());
            //setDefaultCommand(new CameraController());
    }
    /**
     * 
     */
    public void MoveClimber(double speed)
    {
        motor.set(speed);
    }
    public void execute()
    {
    }
    @Override
    public void periodic()
    {
    }
}