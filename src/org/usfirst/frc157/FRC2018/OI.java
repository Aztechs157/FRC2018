// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc157.FRC2018;

import org.usfirst.frc157.FRC2018.commands.*;
import org.usfirst.frc157.FRC2018.subsystems.Lift;
import org.usfirst.frc157.FRC2018.subsystems.Lift.direction;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
//import org.usfirst.frc157.FRC2018.subsystems.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    /*public JoystickButton k1;
    public JoystickButton k2;
    public JoystickButton k3;
    public JoystickButton k4;
    public JoystickButton l2;
    public JoystickButton r2;
    public JoystickButton sE;
    public JoystickButton sT;
    public JoystickButton k11;
    public JoystickButton k12;*/
    public JoystickButton l1;
    public JoystickButton r1;
    public JoystickButton opA;
    public JoystickButton opX;
    public JoystickButton opY;
    public static JoystickButton tankDrive;
    public static JoystickButton arcadeDrive;
    public JoystickButton ControllerSwap;
    public JoystickButton AttackSwap;
    public static JoystickButton mechDrive;
    public JoystickButton lBumper;
    public JoystickButton rBumper;
    
    //public Joystick opBox;
    public Joystick gamePad;
    public Joystick operatorGamePad;
    //public JoystickButton opBoxButtons[][];
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static final int R2 = 3;
    public static final int L2 = 2;
    public static final int LYStick = 1;
    public static final int LXStick = 0;
    public static final int RYStick = 5;
    public static final int RxStick = 4;
    public static final int JoyY = 1;
    public static final int JoyX = 0;
    public static final int Ratk = 1;
    public static final int Latk = 1;
    public static final int LT = 2;
    public static final int RT = 3;
    public static final int LatkPot = 2;
    public static final int RatkPot = 2;
    public static final int RXAttackStick = 0;
    public OI() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        //opBox = new Joystick(1);
        gamePad = new Joystick(3);
        operatorGamePad = new Joystick(2);

      /*  ControllerSwap = new JoystickButton(gamePad, 8);
        ControllerSwap.whileHeld(new DriveSwap());
        mechDrive = new JoystickButton(gamePad, 3);
        tankDrive = new JoystickButton(gamePad, 4);
        arcadeDrive = new JoystickButton(gamePad, 1);
        */
        /*k12 = new JoystickButton(opBox, 12);
        k11 = new JoystickButton(opBox, 11);
        sT = new JoystickButton(opBox, 10);
        sE = new JoystickButton(opBox, 9);
        //r1 = new JoystickButton(opBox, 8);
        //l1 = new JoystickButton(opBox, 7);
        r2 = new JoystickButton(opBox, 6);
        l2 = new JoystickButton(opBox, 5);
        k4 = new JoystickButton(opBox, 4);
        k3 = new JoystickButton(opBox, 3);
        k2 = new JoystickButton(opBox, 2);
        k1 = new JoystickButton(opBox, 1);*/
        l1 = new JoystickButton(operatorGamePad, 5);
        r1 = new JoystickButton(operatorGamePad, 6);
        opA = new JoystickButton(operatorGamePad, 1);
        opX = new JoystickButton(operatorGamePad, 3);
        opY = new JoystickButton(operatorGamePad, 4);
        /*opBoxButtons = new JoystickButton[][]{
                {k1, k2, k3, k4},
                {l2, r2, l1, r1},
                {sE, sT, k11, k12}
        };*/
        l1.whileHeld(new opMoveDown());
        r1.whileHeld(new opMoveUp());
        opA.whileHeld(new MoveLiftToPos(0));
        opX.whileHeld(new MoveLiftToPos(4));
        opY.whileHeld(new MoveLiftToPos(6));
        /*opBoxButtons[0][0].whileHeld(new grabberMove(true));
        opBoxButtons[1][0].whileHeld(new grabberMove(false));
        opBoxButtons[2][0].whileHeld(new Debug());
        opBoxButtons[0][1].whileHeld(new MoveLiftToPos(2));
        opBoxButtons[1][1].whileHeld(new MoveLiftToPos(1));
        opBoxButtons[0][2].whileHeld(new MoveLiftToPos(6));
        opBoxButtons[1][2].whileHeld(new MoveLiftToPos(0));
        opBoxButtons[0][3].whileHeld(new MoveLiftToPos(3));
        opBoxButtons[1][3].whileHeld(new MoveLiftToPos(4));
        opBoxButtons[2][3].whileHeld(new MoveLiftToPos(5));*/
        //left Bumper 5 right Bumper 6 
        lBumper = new JoystickButton(gamePad,5);
        lBumper.whileHeld(new grabberMove(true));
        rBumper = new JoystickButton(gamePad,6);
        rBumper.whileHeld(new grabberMove(false));
        //used for dual joystick
        //AttackSwap = new JoystickButton(attackL, 6);
        //AttackSwap.whenPressed(new AtkSwap());
        // SmartDashboard Buttons
        SmartDashboard.putData("AuotoGroup", new AuotoGroup());
        SmartDashboard.putData("DriveWithSticks", new DriveWithSticks());
        SmartDashboard.putData("MoveLiftToPos1", new MoveLiftToPos(0));
        SmartDashboard.putData("MoveLiftToPos2", new MoveLiftToPos(1));
        SmartDashboard.putData("MoveLiftToPos3", new MoveLiftToPos(2));
        SmartDashboard.putData("MoveLiftToPos4", new MoveLiftToPos(3));
        SmartDashboard.putData("MoveLiftToPos5", new MoveLiftToPos(4));
        SmartDashboard.putData("MoveLiftToPos6", new MoveLiftToPos(5));
        SmartDashboard.putData("MoveLiftToPos7", new MoveLiftToPos(6));
        SmartDashboard.putData("grabberMoveIn", new grabberMove(true));
        SmartDashboard.putData("grabberMoveOut", new grabberMove(false));
        SmartDashboard.putData("empty", new empty());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
    /*public Joystick getopBox() {
        return opBox;
    }*/



    public Joystick getgamePad() {
        return gamePad;
    }
    public Joystick getOperatorGamePad() {
        return operatorGamePad;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
}

