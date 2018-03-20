package org.usfirst.frc157.FRC2018;

import edu.wpi.first.wpilibj.Timer;

public class SlewRate {
	
	private double lastRate;
	private double lastTime;
	private double standardMaxAccel;
	
	public SlewRate(double maxAccel) {
		lastTime = Timer.getFPGATimestamp();
		standardMaxAccel = maxAccel;
		lastRate = 0;
	}
	public double rateCalculate(double desired) {
	    return rateCalculate(desired, standardMaxAccel);
	}
	public double rateCalculate(double desired, double maxAccel) {

		double deltaTime = Timer.getFPGATimestamp()-lastTime;
		double desiredAccel = (desired - lastRate)/deltaTime;
		double addedRate;
		double newRate;
		
		if (Math.abs(desiredAccel) < maxAccel) {
		    addedRate = desiredAccel*deltaTime;
		    newRate = addedRate+lastRate;
		}
		else {
		    addedRate = ((desiredAccel>0)? 1: -1)*maxAccel*deltaTime;
            newRate = addedRate+lastRate;
		}
		lastTime = lastTime+deltaTime;
		lastRate = newRate;

		double returnVal = newRate;
		return returnVal;
	}
	public void reinit() {
		lastTime = Timer.getFPGATimestamp();
		lastRate = 0;
	}
}
