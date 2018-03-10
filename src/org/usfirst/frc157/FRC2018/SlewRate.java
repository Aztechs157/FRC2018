package org.usfirst.frc157.FRC2018;

import edu.wpi.first.wpilibj.Timer;

public class SlewRate {
	
	private double lastRate;
	private double lastTime;
	private double maxAccel;
	
	public SlewRate(double maxAccel) {
		lastTime = Timer.getFPGATimestamp();
		this.maxAccel = maxAccel;
		lastRate = 0;
	}
	public double rateCalculate(double desired) {
		double absDesired = Math.abs(desired);
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
	//	newRate = ((desired>=0)? 1: -1)*newRate;
		lastRate = newRate;
		double returnVal = newRate;
		return returnVal;
	}
	public void reinit() {
		lastTime = Timer.getFPGATimestamp();
		lastRate = 0;
	}
}
