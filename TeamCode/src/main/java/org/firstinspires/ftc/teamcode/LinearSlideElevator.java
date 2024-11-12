package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class LinearSlideElevator{
    private DcMotor grabberArmElevator;
    public static double grabber_current_position = 0.0;
    public static int grabber_max_position = 3800;
    private Telemetry telemetry;


    public LinearSlideElevator(HardwareMap hardwareMap){
        grabberArmElevator = hardwareMap.get(DcMotor.class, "grabberArmElevator");
        grabberArmElevator.setDirection(DcMotorSimple.Direction.FORWARD);
        grabberArmElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public  void ClawUp(){
        int startPosition;
        grabberArmElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber_current_position = grabberArmElevator.getCurrentPosition();
        telemetry.addData("Current Position:", grabber_current_position);

        if (grabber_current_position < grabber_max_position) {
            grabberArmElevator.setPower(1);
            telemetry.addData("Grabber current position:", grabber_current_position);
            telemetry.addData("Power set to: ", 1);
            //grabberArmElevator.setTargetPosition(grabber_max_position);
            //telemetry.addData("Target Position Set To:", 700 );
        } else {
            telemetry.addLine("Max Height Reached");
            grabberArmElevator.setPower(0);
        }
        telemetry.addData("Grabber Max Possition",grabber_max_position);
    }
}