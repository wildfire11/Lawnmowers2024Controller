package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test IKEA", group = "MAIN")
public class ServoTest extends LinearOpMode {
    Servo servo1;
    @Override
    public void runOpMode() throws InterruptedException {
        servo1=hardwareMap.get(Servo.class,"servo");
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.y) {
                telemetry.addLine("Y is pressed");
                telemetry.update();
                servo1.setPosition(1);
            }else{
                servo1.setPosition(0);
                telemetry.addLine("Y is not pressed");
                telemetry.update();
            }
        }
    }
}
