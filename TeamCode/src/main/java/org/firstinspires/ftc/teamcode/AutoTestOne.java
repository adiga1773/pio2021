/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;


/**
 * ((((real))))
 */

@Autonomous(name="Positioning testing", group="Linear Opmode")
//@Disabled
public class AutoTestOne extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive    = null;
    private DcMotor leftBackDrive     = null;
    private DcMotor rightFrontDrive   = null;
    private DcMotor rightBackDrive    = null;
    private DcMotor leftShooterDrive  = null;
    private DcMotor rightShooterDrive = null;
    private DcMotor intakeDrive       = null;
    private DcMotor armMotor          = null;

    private CRServo gripperServo = null;

    boolean shooterActive = false;
    boolean eStop = false;

    private DistanceSensor fwdDistance;
    private DistanceSensor lftDistance;
    private DistanceSensor bckDistance;
    private DistanceSensor rhtDistance;


    //IMU stuff
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    AngularVelocity angularVelocity;

    double lastTime = 0;
    double heading;
    double lastHeading = 0;
    double spinRate;

    long headingAcquisitionTime;
    long lastHeadingAcquisitionTime;

    double targetHeading = 0;

    double fieldX;
    double fieldY;

    double frontRange = 0;
    double leftRange = 0;
    double backRange = 0;
    double rightRange = 0;


    //heading only
    RobotConstant robotConstant = new RobotConstant();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        leftFrontDrive    = hardwareMap.get(DcMotor.class, "front left motor");
        leftBackDrive     = hardwareMap.get(DcMotor.class, "back left motor");
        rightFrontDrive   = hardwareMap.get(DcMotor.class, "front right motor");
        rightBackDrive    = hardwareMap.get(DcMotor.class, "back right motor");

        leftShooterDrive  = hardwareMap.get(DcMotor.class, "left shooter motor");
        rightShooterDrive = hardwareMap.get(DcMotor.class, "right shooter motor");

        intakeDrive       = hardwareMap.get(DcMotor.class, "intake motor");

        armMotor          = hardwareMap.get(DcMotor.class, "arm motor");

        gripperServo      = hardwareMap.get(CRServo.class, "gripper servo");

        fwdDistance = hardwareMap.get(DistanceSensor.class, "forward range");
        lftDistance = hardwareMap.get(DistanceSensor.class, "left range");
        bckDistance = hardwareMap.get(DistanceSensor.class, "rear range");
        rhtDistance = hardwareMap.get(DistanceSensor.class, "right range");

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

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        //imu stuffs
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        frontRange = fwdDistance.getDistance(DistanceUnit.METER);
        leftRange = lftDistance.getDistance(DistanceUnit.METER);
        backRange = bckDistance.getDistance(DistanceUnit.METER);
        rightRange = rhtDistance.getDistance(DistanceUnit.METER);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity  = imu.getGravity();
        angularVelocity = imu.getAngularVelocity();
        runtime.reset();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1);
        time = runtime.time();
        heading = angles.firstAngle;
        headingAcquisitionTime = angles.acquisitionTime;



        targetHeading = 0;

        //starts reading threads
        Thread  twoMDReadThread = new ReadDistance();
        twoMDReadThread.start();

        Thread imuReadThread = new ReadIMU();
        imuReadThread.start();




        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            lastTime = time;
            lastHeading = heading;

            lastHeadingAcquisitionTime = headingAcquisitionTime;
            headingAcquisitionTime = angles.acquisitionTime;

            time = runtime.time();
            heading = angles.firstAngle;
            spinRate = angularVelocity.xRotationRate;




            calculatePosition();

            if(gamepad1.y){
                targetHeading = heading;
                if(gamepad1.dpad_up){
                    targetHeading = 0;
                }
                if(gamepad1.dpad_left){
                    targetHeading = Math.PI * 0.5;
                }
                if(gamepad1.dpad_down){
                    targetHeading = Math.PI;
                }
                if(gamepad1.dpad_right){
                    targetHeading = Math.PI * 1.5;
                }
            }

            //corrects if target behind itself
            if(targetHeading > 1.74533 && targetHeading < 4.88692){
                if(heading < 0){
                    heading += Math.PI * 2;
                }
                if(lastHeading < 0){
                    lastHeading += Math.PI * 2;
                }
                robotConstant.headingsForward = false;
            }
            else{
                if(targetHeading >= 4.88692){
                    targetHeading -= Math.PI * 2;
                }
            }

            double headingError = targetHeading - heading;


            //speed factors affects all but shooters
            double speedFactor = 1.0;
            if (gamepad1.left_bumper) {
                eStop = true;
            }
            if (eStop) speedFactor = 0.0;

            if (gamepad1.right_bumper) {
                eStop = false;
                speedFactor = 0.5;
            }

            // Setup a variable for each drive wheel to save power level for telemetry
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftFrontPower;
            double leftBackPower;
            double rightFrontPower;
            double rightBackPower;
            double shooterPower = 0.0;
            double armPower = 0.0;
            double gripperPower = 0.0;



            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            double drive = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x + -gamepad2.left_stick_x; //gamepad 2 for testing
            double strafe = gamepad1.left_stick_x;

            turn += PIDHeadingControl(headingError);//where da magic happens

            leftFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0) * speedFactor;
            leftBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0) * speedFactor;
            rightFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0) * speedFactor;
            rightBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0) * speedFactor;

            //ensure not less than .11 so it doesn't squeak
            leftFrontPower = ensurePowerHighEnough(leftFrontPower);
            leftBackPower = ensurePowerHighEnough(leftBackPower);
            rightFrontPower = ensurePowerHighEnough(rightFrontPower);
            rightBackPower = ensurePowerHighEnough(rightBackPower);


            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);

            //shooter stuff
            if (gamepad1.a && !gamepad1.right_bumper) {
                shooterActive = true;
            }
            if (shooterActive)
                shooterPower = 1.0;
            if (gamepad1.b && !gamepad1.right_bumper) {
                shooterActive = false;
                shooterPower = -0.5;
            }
            shooterPower *= speedFactor;

            leftShooterDrive.setPower(shooterPower);
            rightShooterDrive.setPower(shooterPower);

            double intakePower = (gamepad1.right_trigger - gamepad1.left_trigger) * speedFactor;
            intakePower = Range.clip(intakePower, -1.0, 1.0);
            intakeDrive.setPower(intakePower);

            //arm
            //arm stuffs
            if (gamepad1.dpad_up) {
                armPower = -1.0;
            }
            if (gamepad1.dpad_down) {
                armPower = 0.5;
            }
            armPower *= speedFactor * speedFactor;
            armMotor.setPower(armPower);

            //gripper stuffs
            if (gamepad1.dpad_left) {
                gripperPower = -1.0;
            } else if (gamepad1.dpad_right) {
                gripperPower = 1.0;
            }
            gripperPower *= speedFactor;
            gripperServo.setPower(gripperPower);

            Position posIMU = imu.getPosition();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Positions", "X: (%.4f), Y: (%.4f)", fieldX, fieldY);
            telemetry.addData("IMUPositions", "X: (%.4f), Y: (%.4f)", posIMU.x, posIMU.y);
            telemetry.addData("Located?", "X Tracked: " + !noX + " Y Tracked: " + !noY);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Readings", "front: (%.4f), left: (%.4f)", frontRange, leftRange);
            telemetry.addData("Readings", "right: (%.4f), back: (%.4f)", rightRange, backRange);
            telemetry.addLine("2MD last read: " + last2MDRReadTime);
            telemetry.addLine("Cycle Time Main: " + (time-lastTime));
            telemetry.addLine("Heading: " + heading);

            try {
                double IMUCycleTime = IMUReads.get(IMUReads.size() - 1).getReadTime() - IMUReads.get(IMUReads.size() - 2).getReadTime();
                double TwoMDCycleTime = TwoMDs.get(TwoMDs.size() - 1).getReadTime() - TwoMDs.get(TwoMDs.size() - 2).getReadTime();
                telemetry.addData("Read Threads Time", "IMU: (%.4f), 2MD: (%.4f)", IMUCycleTime, TwoMDCycleTime);
            }catch (Exception e){
                telemetry.addLine("Reading threads failed");
            }

            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }

        //stops reading threads
        twoMDReadThread.interrupt();
        imuReadThread.interrupt();
    }

    double PIDHeadingControl(double headingError){

        double headingIntegral = headingIntegrator();
        double smoothSpinRate = smoothSpinRate();
        double power = robotConstant.headingKPosition * headingError + robotConstant.headingKIntegral * headingIntegral + robotConstant.headingKDerivative * smoothSpinRate;

        //clipping stuff in case of integral saturation
        double clippedPower = Range.clip(power, -0.5, 0.5);
        boolean saturated = (power != clippedPower);
        boolean signMatch = (headingError * robotConstant.headingIntegral) > 0;
        if(saturated && signMatch){

            power = robotConstant.headingKPosition * headingError + robotConstant.headingKDerivative * smoothSpinRate;
            clippedPower = Range.clip(power, -0.5, 0.5);
            robotConstant.headingIntegral -= robotConstant.headingAddedPart;
        }


        telemetry.addLine("HPError: " + robotConstant.headingKPosition * headingError);
        telemetry.addLine("HIError: " + robotConstant.headingKIntegral * headingIntegral);
        telemetry.addLine("HDError: " + robotConstant.headingKDerivative * smoothSpinRate);

        return clippedPower;
    }

    //the rotation reads from the last half second or 10 reads, whichever is first
    double smoothSpinRate(){
        try {
            ArrayList<IMUValues> IMUClone = new ArrayList<>();

            int imuCloneSize = IMUClone.size(); //makes it thread safe or something idk
            for (int i = imuCloneSize - 1; i >= Math.max(imuCloneSize - 11, 0); i--) {
                IMUClone.add(IMUReads.get(i));
            }

            double spin = IMUClone.get(0).getAngularVelocity().xRotationRate;
            double firstTime = IMUClone.get(0).getReadTime();

            double average = spin;

            int count = 1;
            for (int i = 1; i < IMUClone.size(); i++) {
                if (firstTime - 0.5 > IMUClone.get(i).getReadTime()) {//breaks if older than 0.5 seconds
                    break;
                }
                average = (IMUClone.get(i).getAngularVelocity().xRotationRate + IMUClone.get(i - 1).getAngularVelocity().xRotationRate) * 0.5;
                count++;
            }
            average /= count;

            telemetry.addData("SSR", "Count: (%.4f), Result: (%.4f)", count, average);
            return average;
        }catch (Exception e){
            return spinRate;
        }
    }


    double headingIntegrator(){
        double time = (double) (headingAcquisitionTime-lastHeadingAcquisitionTime) / 1000000000;
        robotConstant.headingAddedPart = (targetHeading - (0.5 * (heading + lastHeading)) )  * time;
        robotConstant.headingIntegral += robotConstant.headingAddedPart;

        telemetry.addLine("Heading PID Cycle Time: " + time);
        return robotConstant.headingIntegral;
    }

    double ensurePowerHighEnough(double motorPower){
        if(Math.abs(motorPower) < 0.12){
            return 0;
        }
        else return motorPower;
    }



    //calculates position through 2m distance sensors, if impossible results uses last working sensor
    boolean latestForward = false;
    boolean latestLeft = false;
    boolean noX = true;
    boolean noY = true;
    public void calculatePosition(){
        double forward = -9999;
        double left = -9999;
        double right = -9999;
        double back = -9999;

        if(heading > -Math.PI/4 && heading < Math.PI/4){//front
            if(frontRange < 2.00){
                forward = frontRange;
            }
            if(leftRange < 2.00){
                left = leftRange;
            }
            if(rightRange < 2.00){
                right = rightRange;
            }
            if(backRange < 2.00){
                back = backRange;
            }
        }
        else if((heading >= -Math.PI/4 && heading < -3 * Math.PI/4) || (heading>= Math.PI * 5/4 && heading <= Math.PI * 7/4)){//right
            if(frontRange < 2.00){
                left = frontRange;
            }
            if(leftRange < 2.00){
                forward = left;
            }
            if(rightRange < 2.00){
                back = rightRange;
            }
            if(backRange < 2.00){
                right = backRange;
            }
        }
        else if(heading >= 3 * Math.PI/4 ||  heading < -3 * Math.PI/4){//back
            if(frontRange < 2.00){
                backRange = frontRange;
            }
            if(leftRange < 2.00){
                right = leftRange;
            }
            if(rightRange < 2.00){
                left = rightRange;
            }
            if(backRange < 2.00){
                forward = backRange;
            }
        }
        else if(heading <= Math.PI/4 && heading > 3 * Math.PI/4){//left
            if(frontRange < 2.00){
                left = frontRange;
            }
            if(leftRange < 2.00){
                back = leftRange;
            }
            if(rightRange < 2.00){
                forward = rightRange;
            }
            if(backRange < 2.00){
                left = backRange;
            }
        }


        if(forward == -9999 && back == -9999){
            noY= true;
        }
        else{
            noY = false;
            if(forward != -9999 && back == -9999){
                fieldY = 3.6576 - forward;
                latestForward = true;
            }
            else if(forward == -9999 && back != -9999){
                fieldY = back;
                latestForward = false;
            }
            else if(latestForward){
                fieldY = 3.6576 - forward;
            }else {
                fieldY = back;
            }
        }

        if(right == -9999 && left == -9999){
            noX = true;
        }
        else {
            noX = false;
            if (right != -9999 && left == -9999) {
                fieldX = 3.6576 - right;
                latestLeft = false;
            } else if (right == -9999 && left != -9999) {
                fieldX = left;
                latestLeft = true;
            } else if (latestLeft) {
                fieldX = left;
            } else {
                fieldX = 3.6576 - right;
            }
        }

    }


    //reads the 2m distance sensors in a separate thread to reduce input lag
    double last2MDRReadTime = 0;
    ArrayList<TwoMDValue> TwoMDs = new ArrayList<>(); //to be used for derivatives and stuff
    private class ReadDistance extends Thread
    {
        @Override
        public void run()
        {

            try
            {
                while (!isInterrupted())
                {
                    // we record the Y values in the main class to make showing them in telemetry
                    // easier.

                    //read 2MD
                    frontRange = fwdDistance.getDistance(DistanceUnit.METER);
                    leftRange = lftDistance.getDistance(DistanceUnit.METER);
                    backRange = bckDistance.getDistance(DistanceUnit.METER);
                    rightRange = rhtDistance.getDistance(DistanceUnit.METER);
                    last2MDRReadTime = (System.nanoTime() / Math.pow(10, 9));

                    TwoMDValue latestRead = new TwoMDValue( frontRange, backRange, leftRange, rightRange, last2MDRReadTime);
                    TwoMDs.add(latestRead);

                    idle();
                }
            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            // an error occurred in the run loop.
            catch (Exception e) {

            }
        }
    }

    double lastIMUReadTime = 0;
    ArrayList<IMUValues> IMUReads = new ArrayList<>();
    Position positionIMU;
    private class ReadIMU extends Thread{
        @Override
        public void run()
        {

            try
            {
                while (!isInterrupted())
                {
                    // we record the Y values in the main class to make showing them in telemetry
                    // easier.

                    //read IMU
                    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                    gravity  = imu.getGravity();
                    angularVelocity = imu.getAngularVelocity();
                    lastIMUReadTime = (System.nanoTime() / Math.pow(10, 9));
                    Position pos = imu.getPosition();
                    positionIMU = pos;

                    IMUValues values = new IMUValues(angles, gravity, angularVelocity, pos, lastIMUReadTime);
                    IMUReads.add(values);

                    idle();
                }
            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            // an error occurred in the run loop.
            catch (Exception e) {

            }
        }
    }
}

