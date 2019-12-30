// Code that implements a camera into the driver sation
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;
import frc.robot.commands.ClimberController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//import frc.robot.commands.CameraController;
//import frc.robot.commands.GrabberWithTriggers;

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