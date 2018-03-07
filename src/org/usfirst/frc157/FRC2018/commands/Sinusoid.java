package org.usfirst.frc157.FRC2018.commands;

public class Sinusoid {
	 private double a;
	 private double b;
	 
	 public Sinusoid(double a, double b) {
		 this.a = a;
		 this.b = b;
	 }
	 public double xSinCalculate(double distance) {
			double sum = 0;
			double curX = 0;
			double calcSlice = 0;
			double deltaX = 0.001;
			while (Math.abs(sum-distance)>0.01) {
				//System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
				calcSlice = Math.sqrt(1+Math.pow(a*b*Math.acos(b*curX),2))*deltaX;
				sum+=(calcSlice);

				curX+=deltaX;
			}
			//System.out.println(calcSlice + "\t\t " + sum + "\t\t" + curX);
			return (curX > a)? a : curX;
		}

	    public double ySinCalculate(double a, double b, double x) {
	        double y = a*Math.sin(b*x);
	        return y;
	    }
	    public double angleSinCalculate(double a, double b, double x) {
			double slope = a*b*Math.acos(b*x);
			double angle = Math.toDegrees(Math.atan(slope));

			return angle;
		}
}
