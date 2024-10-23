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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name=" FieldCentricTeleop ", group="Iterative OpMode")
public class FieldCentricTeleop extends OpMode
{
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    DcMotor armotor;
    Servo servo1;
    DcMotor grabberArmElevator;
    static final int    CYCLE_MS    =   50;     // period of each cycle
    IMU imu = null;
    int max_position = 34250;
    int min_position = 200;
    double current_position = 0.0;
    int grabber_max_position = 700;
    int grabber_min_position = 0;
    double grabber_current_position = 0.0;
    boolean safety_net = true;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Retrieve the IMU from the hardware map
        imu =  hardwareMap.get(IMU.class, "imu");
        servo1=hardwareMap.get(Servo.class,"servo");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
// Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        telemetry.addData("Status", "Initialized");
        if (gamepad1.options) {
            imu.resetYaw();
        }
        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        armotor = hardwareMap.get(DcMotor.class, "armotor");
        armotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armotor.setTargetPosition(0);
        armotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


//THIS IS VERY USEFUL :)
        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        grabberArmElevator = hardwareMap.get(DcMotor.class, "grabberArmElevator");
        grabberArmElevator.setDirection(DcMotorSimple.Direction.FORWARD);
        grabberArmElevator.setTargetPosition(0);
        grabberArmElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//THIS IS VERY USEFUL :)
        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();




        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        current_position = armotor.getCurrentPosition();
        telemetry.addData("Current Position", current_position);
        telemetry.update();
        while (gamepad2.y) {
            armotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            current_position = armotor.getCurrentPosition();
            telemetry.addData("Current Position:", current_position);
            if (current_position < max_position){
                armotor.setPower(0.75);
                telemetry.addData("Power set to: ", 0.25);
                armotor.setTargetPosition(max_position);
                telemetry.addData("Target Position Set To:", 700 );
                telemetry.update();}
            else {
                telemetry.addLine("Max Height Reached");
//                    armotor.setPower(0);
            }

        }
        while (gamepad2.start) {
            armotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            grabberArmElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            grabberArmElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine("ran");
            telemetry.update();



        }
        while (gamepad2.a) {
            int startPosition;
            armotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            current_position = armotor.getCurrentPosition();
            telemetry.addData("Current Position:", current_position);
            if (current_position > min_position){
                armotor.setPower(-0.75);
                telemetry.addData("Power set to: ", -0.25);
                armotor.setTargetPosition(min_position);
                telemetry.addData("Target Position Set To:", 0);
                telemetry.update();}
            else {
                telemetry.addLine("Min Height Reached");
//                    armotor.setPower(0);
            }


        }
        grabber_current_position = grabberArmElevator.getCurrentPosition();
        telemetry.addData("Current Position", grabber_current_position);
        telemetry.update();
        while (gamepad2.x) {
            int startPosition;
            grabber_current_position = grabberArmElevator.getCurrentPosition();
            telemetry.addData("Current Position:", grabber_current_position);
            if (grabber_current_position < grabber_max_position || !safety_net){
                grabberArmElevator.setPower(0.25);
                telemetry.addData("Power set to: ", 0.25);
                //grabberArmElevator.setTargetPosition(grabber_max_position);
                //telemetry.addData("Target Position Set To:", 700 );
                telemetry.update();}
            else {
                telemetry.addLine("Max Height Reached");
                grabberArmElevator.setPower(0);
            }


        }
        while (gamepad2.b) {
            int startPosition;
            grabber_current_position = grabberArmElevator.getCurrentPosition();
            telemetry.addData("Current Position:", grabber_current_position);
            if (grabber_current_position > grabber_min_position || !safety_net){
                grabberArmElevator.setPower(-0.25);
                telemetry.addData("Power set to: ", -0.25);
                grabberArmElevator.setTargetPosition(grabber_min_position);
                //telemetry.addData("Target Position Set To:", 0);
                telemetry.update();}
            else {
                telemetry.addLine("Min Height Reached");
                grabberArmElevator.setPower(0);
            }


            telemetry.update();}
        if (gamepad2.left_bumper) {
            if(safety_net){
                safety_net = false;
                telemetry.addData("Safety Net: ",  safety_net);
                telemetry.update();}
            else{
                safety_net = true;
                telemetry.addData("Safety Net: ",  safety_net);
                telemetry.update();}
        }
        grabberArmElevator.setPower(0);
        if (gamepad2.right_stick_button) {
            telemetry.addLine("opening claw");
            telemetry.update();
            servo1.setPosition(1);
        }
        if(gamepad2.left_stick_button) {
            servo1.setPosition(0);
            telemetry.addLine("closing claw");
            telemetry.update();
        }

        //         armotor.setPower(0);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
