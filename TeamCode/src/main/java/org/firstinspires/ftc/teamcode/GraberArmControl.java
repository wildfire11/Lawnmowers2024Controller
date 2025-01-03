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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "left_drive" as is found on a Robot.
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name = "Graber Arm Teleop Controls", group = "MAIN")
public class GraberArmControl extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    //THIS IS VERY USEFUL :)
    // Define class members
    DcMotor grabberArmElevator;





    @Override
    public void runOpMode() {

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        grabberArmElevator = hardwareMap.get(DcMotor.class, "grabberArmElevator");
        grabberArmElevator.setDirection(DcMotorSimple.Direction.FORWARD);
        //grabberArmElevator.setTargetPosition(0);
        grabberArmElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//THIS IS VERY USEFUL :)
        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        waitForStart();
//THIS IS VERY USEFUL :)
        // Ramp motor speeds till stop pressed.

        int grabber_max_position = 2100;
        int grabber_min_position = 0;
        double grabber_current_position = 0.0;

        boolean safety_net = true;


        while(opModeIsActive()) {
//THIS IS VERY USEFUL :)
            // Ramp the motors, according to the rampUp variable.
//            if (rampUp) {
//                // Keep stepping up until we hit the max value.
//                power += INCREMENT ;
//                if (power >= MAX_FWD ) {
//                    power = MAX_FWD;
//                    rampUp = !rampUp;   // Switch ramp direction
//                }
//            }
//            else {
//                //THIS IS VERY USEFUL :)
//                // Keep stepping down until we hit the min value.
//                power -= INCREMENT ;
//                if (power <= MAX_REV ) {
//                    power = MAX_REV;
//                    rampUp = !rampUp;  // Switch ramp direction
//                }
//            }2

            grabber_current_position = grabberArmElevator.getCurrentPosition();
            telemetry.addData("Current Position", grabber_current_position);
            telemetry.update();
            while (gamepad1.y) {
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
            while (gamepad1.a) {
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
            if (gamepad1.x) {
                if(safety_net){
                    safety_net = false;
                    telemetry.addData("Safety Net: ",  safety_net);
                    telemetry.update();}
                else{
                    safety_net = true;
                    telemetry.addData("Safety Net: ",  safety_net);
                    telemetry.update();}
            }


            if (gamepad1.start) {
                grabberArmElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                grabberArmElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addLine("ran");
                telemetry.update();
            }
            grabberArmElevator.setPower(0);


            sleep(CYCLE_MS);
            idle();
        }

        // Turn off motor and signal done;
//        grabberArmElevator.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
