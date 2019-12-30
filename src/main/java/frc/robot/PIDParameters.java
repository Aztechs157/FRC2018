package frc.robot;

/** 
 * Passed to a PID class
 */
public class PIDParameters {
    public double kP;
    public double kI;
    public double kD;
    public double rangeIStart;
    public double rangeIEnd;
    public double rangeDStart;
    public double rangeDEnd;

    /**
     * 
     * @param kP
     * @param kI
     * @param kD
     * @param rangeIStart 
     * @param rangeIEnd
     * @param rangeDStart Furthest from desiredPosition
     * @param rangeDEnd Closest to desiredPosition
     */
    public PIDParameters(double kP, double kI, double kD, double rangeIStart, double rangeIEnd, double rangeDStart, double rangeDEnd) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.rangeIStart = rangeIStart;
        this.rangeIEnd = rangeIEnd;
        this.rangeDStart = rangeDStart;
        this.rangeDEnd = rangeDEnd;
    }

	public PIDParameters(double kP2, double kI2, double kD2, double rangeIStart2, double rangeIEnd2) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.rangeIStart = rangeIStart;
        this.rangeIEnd = rangeIEnd;
	}
}
