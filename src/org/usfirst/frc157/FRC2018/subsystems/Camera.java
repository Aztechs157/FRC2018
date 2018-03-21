// Code that implements a camera into the driver sation
package org.usfirst.frc157.FRC2018.subsystems;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc157.FRC2018.commands.CameraController;
import org.usfirst.frc157.FRC2018.commands.GrabberWithTriggers;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
/**
 *  @AUTH Connor Hurley.
 */
public class Camera extends Subsystem {
    public boolean debugMode = true; // Change to true to view camera stats
    /**
     * Defines cam0 as an HTTP camera on ShuffleBoard
     */
    public UsbCamera cam0;
    @Override
    public void initDefaultCommand()
    {
            cam0 = CameraServer.getInstance().startAutomaticCapture();
            cam0.setResolution(640, 480); // (640, 480), (320, 240) (160, 120)
            cam0.setFPS(15);
            if(!cam0.isConnected()) System.err.println("CAMERA IS NOT PLUGGED IN!");
            setDefaultCommand(new CameraController());
    }
    /**
     * execute() is to be used to add filters on the camera
     */
    public void execute()
    {
    }
    @Override
    public void periodic()
    {
    }
}