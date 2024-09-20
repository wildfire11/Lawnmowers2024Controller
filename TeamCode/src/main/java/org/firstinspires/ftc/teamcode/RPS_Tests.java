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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "left_drive" as is found on a Robot.
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Challenge: refactor this code to test all 4 motors, one at a time.
 */
@TeleOp(name = "RPS Test", group = "Lawnmower Tests")
public class RPS_Tests extends LinearOpMode {


    // Define class members

    double  power   = 0;
    boolean rampUp  = true;

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;



    @Override
    public void runOpMode() {

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftRear");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backRightMotor = hardwareMap.get(DcMotor.class, "rearRight");

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a)
            {
                int startPosition;
                startPosition = frontRightMotor.getCurrentPosition();
                telemetry.addData("start position", startPosition);

                frontRightMotor.setPower(1);
                sleep(10000);
                frontRightMotor.setPower(0);

                int EndPossition;
                EndPossition = frontRightMotor.getCurrentPosition();
                telemetry.addData("end position", EndPossition);
                telemetry.addData("distance traveled", EndPossition-startPosition);
                int tps=((EndPossition-startPosition)/10);
                telemetry.addData("tps", tps);
                telemetry.update();
            }
            if (gamepad1.x)
            {
                int startPosition;
                startPosition = frontLeftMotor.getCurrentPosition();
                telemetry.addData("start position", startPosition);

                frontLeftMotor.setPower(1);
                sleep(10000);
                frontLeftMotor.setPower(0);

                int EndPossition;
                EndPossition = frontLeftMotor.getCurrentPosition();
                telemetry.addData("end position", EndPossition);
                telemetry.addData("distance traveled", EndPossition-startPosition);
                int tps=((EndPossition-startPosition)/10);
                telemetry.addData("tps", tps);
                telemetry.update();
            }

            if (gamepad1.b)
            {
                int startPosition;
                startPosition = backLeftMotor.getCurrentPosition();
                telemetry.addData("start position", startPosition);

                backLeftMotor.setPower(1);
                sleep(10000);
                backLeftMotor.setPower(0);

                int EndPossition;
                EndPossition = backLeftMotor.getCurrentPosition();
                telemetry.addData("end position", EndPossition);
                telemetry.addData("distance traveled", EndPossition-startPosition);
                int tps=((EndPossition-startPosition)/10);
                telemetry.addData("tps", tps);
                telemetry.update();
            }if (gamepad1.y)
            {
                int startPosition;
                startPosition = backRightMotor.getCurrentPosition();
                telemetry.addData("start position", startPosition);

                backRightMotor.setPower(1);
                sleep(10000);
                backRightMotor.setPower(0);

                int EndPossition;
                EndPossition = backRightMotor.getCurrentPosition();
                telemetry.addData("end position", EndPossition);
                telemetry.addData("distance traveled", EndPossition-startPosition);
                int tps=((EndPossition-startPosition)/10);
                telemetry.addData("tps", tps);
                telemetry.update();
            }
        }
        // Ramp motor speeds till stop pressed.

        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

    }}

