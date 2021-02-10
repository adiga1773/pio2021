package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareFirstbot {
    /* Public OpMode members. */

    public DcMotor leftFrontDrive    = null;
    public DcMotor leftBackDrive     = null;
    public DcMotor rightFrontDrive   = null;
    public DcMotor rightBackDrive    = null;
    public DcMotor leftShooterDrive  = null;
    public DcMotor rightShooterDrive = null;
    public DcMotor intakeDrive       = null;
    public DcMotor armMotor          = null;



    public CRServo gripperServo = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareFirstbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        hwMap = this.hwMap;

        leftFrontDrive    = hwMap.get(DcMotor.class, "front left motor");
        leftBackDrive     = hwMap.get(DcMotor.class, "back left motor");
        rightFrontDrive   = hwMap.get(DcMotor.class, "front right motor");
        rightBackDrive    = hwMap.get(DcMotor.class, "back right motor");

        leftShooterDrive  = hwMap.get(DcMotor.class, "left shooter motor");
        rightShooterDrive = hwMap.get(DcMotor.class, "right shooter motor");

        intakeDrive       = hwMap.get(DcMotor.class, "intake motor");

        armMotor          = hwMap.get(DcMotor.class, "arm motor");

        gripperServo      = hwMap.get(CRServo.class, "gripper servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftShooterDrive.setDirection(DcMotor.Direction.REVERSE);
        rightShooterDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeDrive.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }


}
