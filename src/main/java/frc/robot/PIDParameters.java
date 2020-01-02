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
    public double minI;
    public double maxI;
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

	public PIDParameters(double kP, double kI, double kD, double rangeIStart, double rangeIEnd, double rangeDStart, double rangeDEnd, double minI, double maxI) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.rangeIStart = rangeIStart;
        this.rangeIEnd = rangeIEnd;
        this.rangeDStart = rangeDStart;
        this.rangeDEnd = rangeDEnd;
        this.minI = minI;
        this.maxI = maxI;
	}
}
