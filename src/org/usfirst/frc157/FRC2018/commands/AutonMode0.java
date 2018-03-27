
package org.usfirst.frc157.FRC2018.commands;

import org.usfirst.frc157.FRC2018.Robot;
import org.usfirst.frc157.FRC2018.RobotMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;

public class AutonMode0 extends Command
{
    public AutonMode0() {
    }
    public void execute() {
        System.out.println(Robot.drive.getAngle());
        RobotMap.driveRight1.setNeutralMode(NeutralMode.Coast);
        RobotMap.driveRight2.setNeutralMode(NeutralMode.Coast);
        RobotMap.driveLeft1.setNeutralMode(NeutralMode.Coast);
        RobotMap.driveLeft2.setNeutralMode(NeutralMode.Coast);
    }
    public boolean isFinished()
    {
        return false;
    }
}
