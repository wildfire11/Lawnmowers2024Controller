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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
@Disabled
@TeleOp(name = "Ramp Motor Speed Tests", group = "Lawnmower Tests")
public class MotorRampTests extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

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
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftRear");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightRear");

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {
                while(gamepad1.x) {
                    // Ramp the motors, according to the rampUp variable.
                    if (rampUp) {
                        // Keep stepping up until we hit the max value.
                        power += INCREMENT;
                        if (power >= MAX_FWD) {
                            power = MAX_FWD;
                            rampUp = !rampUp;   // Switch ramp direction
                        }
                    } else {
                        // Keep stepping down until we hit the min value.
                        power -= INCREMENT;
                        if (power <= MAX_REV) {
                            power = MAX_REV;
                            rampUp = !rampUp;  // Switch ramp direction

                        }
                    }

                    // Display the current value
                    telemetry.addData("frontleft motor power", "%5.2f", power);
                    telemetry.addData(">", "Press Stop to end test.");
                    telemetry.update();

                    // Set the motor to the new power and pause;
                    frontLeftMotor.setPower(power);
                    sleep(CYCLE_MS);
                    idle();
                }
                while(gamepad1.b) {
                    if (rampUp) {
                        // Keep stepping up until we hit the max value.
                        power += INCREMENT;
                        if (power >= MAX_FWD) {
                            power = MAX_FWD;
                            rampUp = !rampUp;   // Switch ramp direction
                        }
                    } else {
                        // Keep stepping down until we hit the min value.
                        power -= INCREMENT;
                        if (power <= MAX_REV) {
                            power = MAX_REV;
                            rampUp = !rampUp;  // Switch ramp direction

                        }
                    }

                    // Display the current value
                    telemetry.addData("back right motor power", "%5.2f", power);
                    telemetry.addData(">", "Press Stop to end test.");
                    telemetry.update();

                    // Set the motor to the new power and pause;
                    backRightMotor.setPower(power);
                    sleep(CYCLE_MS);
                    idle();
                }
            while(gamepad1.y) {
                if (rampUp) {
                    // Keep stepping up until we hit the max value.
                    power += INCREMENT;
                    if (power >= MAX_FWD) {
                        power = MAX_FWD;
                        rampUp = !rampUp;   // Switch ramp direction
                    }
                } else {
                    // Keep stepping down until we hit the min value.
                    power -= INCREMENT;
                    if (power <= MAX_REV) {
                        power = MAX_REV;
                        rampUp = !rampUp;  // Switch ramp direction

                    }
                }

                // Display the current value
                telemetry.addData("front right motor power", "%5.2f", power);
                telemetry.addData(">", "Press Stop to end test.");
                telemetry.update();

                // Set the motor to the new power and pause;
               frontRightMotor.setPower(power);
                sleep(CYCLE_MS);
                idle();
            }
            while(gamepad1.a) {
                if (rampUp) {
                    // Keep stepping up until we hit the max value.
                    power += INCREMENT;
                    if (power >= MAX_FWD) {
                        power = MAX_FWD;
                        rampUp = !rampUp;   // Switch ramp direction
                    }
                } else {
                    // Keep stepping down until we hit the min value.
                    power -= INCREMENT;
                    if (power <= MAX_REV) {
                        power = MAX_REV;
                        rampUp = !rampUp;  // Switch ramp direction

                    }
                }
                while (gamepad1.y)
                {
                    int startPosition;
                    startPosition = backLeftMotor.getCurrentPosition();
                    telemetry.addData("start position", startPosition);

                    backLeftMotor.setPower(1);
                    sleep(10000);
                    backLeftMotor.setPower(0);}

                // Display the current value
                telemetry.addData("back left motor power", "%5.2f", power);
                telemetry.addData(">", "Press Stop to end test.");
                telemetry.update();

                // Set the motor to the new power and pause;
                backLeftMotor.setPower(power);
                sleep(CYCLE_MS);
                idle();
            }
            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);



        }

        // Turn off motor and signal done;
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();

    }


}
