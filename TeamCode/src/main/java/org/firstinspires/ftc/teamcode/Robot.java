package org.firstinspires.ftc.teamcode;
//class for PID headings so its not hard to change
public class Robot {
    /*
     * Heading Constants
     * Competition Robot kP: 0.7, kI: 0.64, kD: 0.09
     * Muradbot™ kP: 2, kI: 0.50, kD: 0.20 (PROPER TESTING PENDING)
     */
    //Constants are tuned for radians!
    public static double headingKP = 2.00;
    public static double headingKI = 0.50;
    public static double headingKD = 0.20;
    public boolean headingIntegralClamped = false;
    public double headingIntegral = 0;
    public double headingAddedPart;
    public boolean headingsForward = true;
/*
    public double leftZero = 0;
    public double frontZero = 0;
    public double backZero = 0;
    public double rightZero = 0;

 */

}
