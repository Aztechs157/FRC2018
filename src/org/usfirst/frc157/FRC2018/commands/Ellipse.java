package org.usfirst.frc157.FRC2018.commands;

public class Ellipse {
	private double a;
	private double b;
	private int quadrant;
	
	public Ellipse(double a, double b) {
		this.a = a;
		this.b = b;
	}
	public double xEllipseCalculate(double distance) {
    	double semiperimeter = Math.PI*Math.sqrt((a*a+b*b)/2)/2.0;
		quadrant = (int)(distance/semiperimeter)+1;
		System.out.println(quadrant);
		distance = distance-((quadrant-1)*semiperimeter);
		if (quadrant == 2||quadrant == 4) {
			distance = semiperimeter-distance;
		}
		System.out.println(distance);
		double origA = a;
		double tempB = b / a;
		distance = distance / a;
		double tempA = 1;
		double sum = 0;
		double curX = 0;
		double calcSlice = 0;
		double deltaX = 0.001;
		while (Math.abs(sum - distance) > 0.001) {
			// System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
			calcSlice = Math.sqrt(1 + Math.pow(-curX/(tempA*Math.sqrt(tempA*tempA-curX*curX)), 2)) * deltaX;
			sum += (calcSlice);

			curX += deltaX;
		}
		// System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
		double returnVal = (curX > tempA) ? tempA*origA : curX*origA;
		return (quadrant == 3||quadrant == 4)? -returnVal: returnVal;
	}

	public double yEllipseCalculate(double x) {
		double origA = a;
		double tempB = b / a;
		double tempX = x / a;
		double tempA = 1;
		double y = Math.sqrt((1 * tempB * tempB - tempB * tempB * tempX * tempX) / (tempA * tempA));
		return origA*y;
	}

	public double angleEllipseCalculate(double x) {
		x = Math.abs(x);
		double tempB = b / a;
		double tempX = x / a;
		double tempA = 1;
		double slope = -tempX/(Math.sqrt(1-tempX*tempX));
		double angle = Math.toDegrees(Math.atan(slope)); //((quadrant==1||quadrant==3)? 1:-1)
		if (quadrant == 2) {
			angle = ((angle>=0)? 1: -1) *(180-Math.abs(angle));
		}
		return angle;
	}
}
