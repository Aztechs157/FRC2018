package frc.robot;
/*
    Original code by Tim Wescott
    Source: https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf
    Modifications by Ty Silva
    Modified to work in Java and match previous PID code.
*/

import edu.wpi.first.wpilibj.Timer;

public class PID_Wescott {
    private PIDParameters[] optionSets;
    private double lastTime;
    private double lastPosition; // Last position input
    private double sigma; // Integrator state
	private int count;
    double Highestvelocity = 0;//find highest velocity
    
    /**
     * Object to caculate PID
     * @param optionSets a array of PIDParameters
     */
	public PID_Wescott(PIDParameters[] optionSets) {
		this.optionSets = optionSets;
		this.lastTime = Timer.getFPGATimestamp();
		this.lastPosition = 0;
		this.sigma = 0;
	}
	/**
	 * Overload for backwords compatability
	 * Check PIDParameters for usage of these params
	 */
    public PID_Wescott(double kP, double kI, double kD, double rangeIStart, double rangeIEnd,
                       double rangeDStart, double rangeDEnd, double integratMax, double integratMin) {
		this.optionSets = new PIDParameters[] { 
			new PIDParameters(kP, kI, kD, rangeIStart, rangeIEnd, rangeDStart, rangeDEnd, integratMax, integratMin),
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
    public double pidCalculate(double desiredPosition, double CurrentPosition, int optionIndex){
        double kP = optionSets[optionIndex].kP; //LEAVE HERE, DISCUSS WITH STUDENTS
        double kI = optionSets[optionIndex].kI;
        double kD = optionSets[optionIndex].kD;
        double integratMin = optionSets[optionIndex].minI;
        double integratMax = optionSets[optionIndex].maxI;

        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;
        double distanceLeft = desiredPosition-CurrentPosition;

        double pTerm, dTerm, iTerm;

        double powerOutput;
        

        pTerm = kP * distanceLeft; // calculate the proportional term
        // calculate the integral state with appropriate limiting
        sigma += distanceLeft;
        // Limit the integrator state if necessary
        if (sigma < integratMin)
        {
            sigma = integratMin;
        }
        else if (sigma > integratMax)
        {
            sigma = integratMax;
        }
        // calculate the integral term
        iTerm = kI * sigma;
        // calculate the derivative
        dTerm = kD * ((CurrentPosition - lastPosition) / deltaTime);

	System.out.println("sigma: " + sigma + "\nDeltaT: " + deltaTime);
        lastPosition = CurrentPosition;
        powerOutput = pTerm + iTerm - dTerm;
        
        if (powerOutput > 1) powerOutput = 1;
		else if (powerOutput < -1) powerOutput = -1;

		return powerOutput;
    }
}