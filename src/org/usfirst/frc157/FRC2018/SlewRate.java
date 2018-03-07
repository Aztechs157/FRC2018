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
		System.out.println("Delta Time: "+ deltaTime);
		double desiredAccel = (absDesired - lastRate)/deltaTime;
		double addedRate;
		double newRate;
		if(desiredAccel>maxAccel) { 
			addedRate = maxAccel*deltaTime;
			newRate = addedRate+lastRate;
		}
		else {
			newRate = absDesired;
		}
		lastTime = lastTime+deltaTime;
		lastRate = newRate;
		double returnVal = ((desired>=0)? 1: -1)*newRate;
		System.out.println("Return: "+returnVal);
		return returnVal;
	}
	public void reinit() {
		lastTime = Timer.getFPGATimestamp();
		lastRate = 0;
	}
}
