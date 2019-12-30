package frc.robot;

//use these equations for the lift and stage, in that order
//https://www.desmos.com/calculator/r9dcoor1vc
public class Deceleration
{
    private double add = 0;
    private double top = 0;
    public Deceleration(double stop, double rate)
    {
        this.top = rate;
        this.add = stop+rate;
    }
    public double calcSpeed(double currentpos)
    {
        return (this.top/(currentpos-this.add))+1;
    }
}