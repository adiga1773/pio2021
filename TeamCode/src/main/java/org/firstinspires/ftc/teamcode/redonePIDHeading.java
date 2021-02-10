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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**

 */

@TeleOp(name="PID HEADING", group="Iterative Opmode")
//@Disabled
public class redonePIDHeading extends OpMode
{
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


    //heading only
    RobotConstant headingPIDConstants = new RobotConstant();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity  = imu.getGravity();
        angularVelocity = imu.getAngularVelocity();
        runtime.reset();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1);
        time = runtime.time();
        heading = angles.firstAngle;
        headingAcquisitionTime = angles.acquisitionTime;

        targetHeading = 0;

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //read IMU
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity  = imu.getGravity();
        angularVelocity = imu.getAngularVelocity();

        lastTime = time;
        lastHeading = heading;

        lastHeadingAcquisitionTime = headingAcquisitionTime;
        headingAcquisitionTime = angles.acquisitionTime;

        time = runtime.time();
        heading = angles.firstAngle;
        spinRate = angularVelocity.xRotationRate;

        if(gamepad1.x){
            targetHeading = heading;
        }
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

        if(targetHeading > 1.74533 && targetHeading < 4.88692){
            if(heading < 0){
                heading += Math.PI * 2;
            }
            if(lastHeading < 0){
                lastHeading += Math.PI * 2;
            }
            headingPIDConstants.headingsForward = false;
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


        //PID tuning stuff
        if(gamepad2.x){//p
            if(gamepad2.dpad_up){
                headingPIDConstants.headingKPosition += 0.01;
            }
            if(gamepad2.dpad_down){
                headingPIDConstants.headingKPosition -= 0.01;
            }
        }
        if(gamepad2.y){//i
            if(gamepad2.dpad_up){
                headingPIDConstants.headingKIntegral += 0.01;
            }
            if(gamepad2.dpad_down){
                headingPIDConstants.headingKIntegral -= 0.01;
            }
        }
        if(gamepad2.b){//d
            if(gamepad2.dpad_up){
                headingPIDConstants.headingKDerivative += 0.01;
            }
            if(gamepad2.dpad_down){
                headingPIDConstants.headingKDerivative -= 0.01;
            }
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "turn (%.2f), right wheel (%.2f)", turn, rightFrontPower);
        telemetry.addData("target and heading", "heading (%.3f), target (%.3f)", heading, targetHeading);
        telemetry.addData("sensors", "rot rate (%.3f), heading Int (%.3f)", spinRate, headingPIDConstants.headingIntegral );
        telemetry.addLine("K values: kP:" + headingPIDConstants.headingKPosition + " kI:" + headingPIDConstants.headingKIntegral + " kD:" + headingPIDConstants.headingKDerivative);
    }


    double PIDHeadingControl(double headingError){
        double headingIntegral = headingIntegrator();
        double power = headingPIDConstants.headingKPosition * headingError + headingPIDConstants.headingKIntegral * headingIntegral + headingPIDConstants.headingKDerivative * spinRate;

        //clipping stuff in case of integral saturation
        double clippedPower = Range.clip(power, -0.5, 0.5);
        boolean saturated = (power != clippedPower);
        boolean signMatch = (headingError * headingPIDConstants.headingIntegral) > 0;
        if(saturated && signMatch){

            power = headingPIDConstants.headingKPosition * headingError + headingPIDConstants.headingKDerivative * spinRate;
            clippedPower = Range.clip(power, -0.5, 0.5);
            headingPIDConstants.headingIntegral -= headingPIDConstants.headingAddedPart;
        }


        telemetry.addLine("HPError: " + headingPIDConstants.headingKPosition * headingError);
        telemetry.addLine("HIError: " + headingPIDConstants.headingKIntegral * headingIntegral);
        telemetry.addLine("HDError: " + headingPIDConstants.headingKDerivative * spinRate);

        return clippedPower;
    }


    double headingIntegrator(){
        double time = (double) (headingAcquisitionTime-lastHeadingAcquisitionTime) / 1000000000;
        headingPIDConstants.headingAddedPart = (targetHeading - (0.5 * (heading + lastHeading)) )  * time;
        headingPIDConstants.headingIntegral += headingPIDConstants.headingAddedPart;

        telemetry.addLine("CYCLE TIME: " + time);
        return headingPIDConstants.headingIntegral;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    double ensurePowerHighEnough(double motorPower){
        if(Math.abs(motorPower) < 0.12){
            return 0;
        }
        else return motorPower;
    }
}
