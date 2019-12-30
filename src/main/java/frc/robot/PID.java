package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.PIDParameters;

public class PID {
	
	private PIDParameters[] optionSets;
	private double lastTime;
	private double lastPosition;  
	private double sigma;
	private int count;
	double Highestvelocity = 0;//find highest velocity

	/**
	 * Object to caculate PID
	 * @param optionSets a array of PIDParameters
	 */
	public PID(PIDParameters[] optionSets) {
		this.optionSets = optionSets;
		this.lastTime = Timer.getFPGATimestamp();
		this.lastPosition = 0;
		this.sigma = 0;
	}
	/**
	 * Overload for backwords compatability
	 * Check PIDParameters for usage of these params
	 */
	public PID(double kP, double kI, double kD, double rangeIStart, double rangeIEnd, double rangeDStart, double rangeDEnd) {
		this.optionSets = new PIDParameters[] { 
			new PIDParameters(kP, kI, kD, rangeIStart, rangeIEnd, rangeDStart, rangeDEnd),
		};
		this.lastTime = Timer.getFPGATimestamp();
		this.lastPosition = 0;
		this.sigma = 0;
	}

	/**
	 * Overload for backwords compatability
	 * If no optionIndex is passed assume it's 0
	 * @param desiredPosition Where you want to go
	 * @param currentPosition Where you are now
	 * @return Calculated power output
	 */
	public double pidCalculate(double desiredPosition, double currentPosition) {
		return pidCalculate(desiredPosition, currentPosition, 0);
	}
	/**
	 * Calculate a power output using pid
	 * @param desiredPosition Where you want to go
	 * @param currentPosition Where you are now
	 * @param optionIndex Which option set to use
	 * @return Calculated power output
	 */
	public double pidCalculate(double desiredPosition, double currentPosition, int optionIndex) {
		PIDParameters options = optionSets[optionIndex];
		double currentTime = Timer.getFPGATimestamp();
		double deltaTime = currentTime - lastTime;
		double deltaPosition = currentPosition - lastPosition;
		double distanceLeft = desiredPosition - currentPosition;
		double velocity = 0;
		
		double powerOutput;


		lastTime = currentTime;
			
		if ((deltaTime > 0) && (Math.abs(distanceLeft) < options.rangeDStart) && (Math.abs(distanceLeft) > options.rangeDEnd))
			velocity = deltaPosition / deltaTime;
		else
			velocity = 0;
	if (Math.abs(velocity) > Math.abs(Highestvelocity))
	{
		Highestvelocity = velocity;
	}
    //System.out.println(Highestvelocity);
		lastPosition = currentPosition;

		if ((Math.abs(distanceLeft) < options.rangeIStart) && (Math.abs(distanceLeft) > options.rangeIEnd))
			sigma += distanceLeft * deltaTime;
		else
			sigma = 0;
	System.out.println("sigma: " + sigma + "\nDeltaT: " + deltaTime);
		powerOutput  = distanceLeft * options.kP;
		powerOutput += sigma * options.kI;
		powerOutput -= velocity * options.kD;	
	

		if (powerOutput > 1) powerOutput = 1;
		else if (powerOutput < -1) powerOutput = -1;

		return powerOutput;
	}
	public double pidCalculate(double desiredPosition, double currentPosition, int optionIndex, boolean printout) {
		if (count == 100){ 
			//System.out.println("DistanceLeft: " + (desiredPosition-currentPosition));
			count = 0;
		}
		count++;
		return pidCalculate(desiredPosition, currentPosition, optionIndex);
	}
}
