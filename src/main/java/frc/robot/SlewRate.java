package frc.robot;

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
	public double getLastTime()
    {
        return lastTime;
    }
    public void setLastTime(double lastTime)
    {
        this.lastTime = lastTime;
    }
    public void reinit() {
		lastTime = Timer.getFPGATimestamp();
		lastRate = 0;
	}
    public double getLastRate()
    {
        return lastRate;
    }
    public void setLastRate(double lastRate)
    {
        this.lastRate = lastRate;
    }
}
