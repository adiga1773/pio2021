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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Simple Working Control", group="Iterative Opmode")
@Disabled
public class redoneControl extends OpMode
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
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
        double turn = -gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;


        leftFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0) * speedFactor;
        leftBackPower = Range.clip(drive - turn - strafe, -1.0, 1.0) * speedFactor;
        rightFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0) * speedFactor;
        rightBackPower = Range.clip(drive + turn + strafe, -1.0, 1.0) * speedFactor;


        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);

        //shooter stuff
        if (gamepad1.a) {
            shooterActive = true;
        }
        if (shooterActive)
            shooterPower = 1.0;
        if (gamepad1.b) {
            shooterActive = false;
            shooterPower = -0.5;
        }
        if(eStop){
            shooterPower = 0.0;
        }

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


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Powers", "drive (%.2f), turn (%.2f)", drive, turn);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
