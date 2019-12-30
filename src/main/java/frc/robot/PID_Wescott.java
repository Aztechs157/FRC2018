package frc.robot;
/*
    Original code by Tim Wescott
    Source: https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf
    Modifications by Ty Silva
    Modified to work in Java.
*/
public class PID_Wescott 
{
    double derState; // Last position input
    double integratState; // Integrator state
    double integratMax, // Maximum and minimum
           integratMin; // allowable integrator state
    double integratGain, // integral gain
           propGain, // proportional gain
           derGain; // derivative gain
    
    double UpdatePID(double error, double position)
    {
        double pTerm, dTerm, iTerm;
        pTerm = propGain * error; // calculate the proportional term
        // calculate the integral state with appropriate limiting
        integratState += error;
        // Limit the integrator state if necessary
        if (integratState > integratMax)
        {
            integratState = integratMax;
        }
        else if (integratState < integratMin)
        {
            integratState = integratMin;
        }
        // calculate the integral term
        iTerm = integratGain * integratState;
        // calculate the derivative
        dTerm = derGain * (derState - position);
        derState = position;
        return pTerm + dTerm + iTerm;
    }
}