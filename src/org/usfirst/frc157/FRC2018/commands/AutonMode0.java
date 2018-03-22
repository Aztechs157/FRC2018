
package org.usfirst.frc157.FRC2018.commands;

import org.usfirst.frc157.FRC2018.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class AutonMode0 extends Command
{
	public AutonMode0() {
		
	}
	public void execute() {
		System.out.println(Robot.drive.getAngle());
	}
	public boolean isFinished()
    {
        return false;
    }
}
