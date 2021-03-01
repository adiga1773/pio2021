package org.firstinspires.ftc.teamcode;
//constructor for PID headings so its not hard to change
public class RobotConstant {
    //heading only kP: 0.7, kI: 0.64, kD: 0.09
    //make sure its in radians!
    public double headingKPosition = 0.70;
    public double headingKIntegral = 2.74;
    public double headingKDerivative = -0.11;
    public boolean headingIntegralClamped = false;
    public double headingIntegral = 0;
    public double headingAddedPart;
    public boolean headingsForward = true;

    public double leftZero = 0;
    public double frontZero = 0;
    public double backZero = 0;
    public double rightZero = 0;

    /* Constructor */
    public RobotConstant(){

    }


}
