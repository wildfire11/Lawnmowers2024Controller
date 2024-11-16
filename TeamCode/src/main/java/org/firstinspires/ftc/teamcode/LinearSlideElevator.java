package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LinearSlideElevator{
    private DcMotor grabberArmElevator;
    public static double grabber_current_position = 0.0;
    public static int grabber_max_position = 3800;
    private Telemetry _telemetry;
    public int grabber_min_position = 0;
    public boolean safety_net = true;
    Servo servo1;
    public static int clawReadyToPullHeight = 2800;
    public boolean isRunningToPosition = false;
    public void stopMovingIfNotTarget() {
        if (isRunningToPosition) {
            grabber_current_position = grabberArmElevator.getCurrentPosition();
            double grabber_target_position = grabberArmElevator.getTargetPosition();
            if(grabber_current_position <= grabber_target_position + 50 && grabber_current_position >= grabber_target_position - 50) {
                grabberArmElevator.setPower(0);
                isRunningToPosition = false;
            }
        } else { grabberArmElevator.setPower(0);

        }
    }



    public LinearSlideElevator(HardwareMap hardwareMap, Telemetry telemetry){
        grabberArmElevator = hardwareMap.get(DcMotor.class, "grabberArmElevator");
        grabberArmElevator.setDirection(DcMotorSimple.Direction.FORWARD);
        grabberArmElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        servo1 = hardwareMap.get(Servo.class, "servo");
        _telemetry = telemetry;
    }
    public void clawToFloor(){
        grabberArmElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabberArmElevator.setTargetPosition(0);
        grabberArmElevator.setPower(1);
    }
    public void clawUp(){
        grabberArmElevator.setTargetPosition(2665);
        grabberArmElevator.setPower(1);
        grabberArmElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void clawReadyToPull(){
        grabberArmElevator.setTargetPosition(clawReadyToPullHeight);
        grabberArmElevator.setPower(1);
        grabberArmElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public  void ClawUp(){
        int startPosition;
        grabberArmElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber_current_position = grabberArmElevator.getCurrentPosition();
        _telemetry.addData("Current Position:", grabber_current_position);

        if (grabber_current_position < grabber_max_position) {
            grabberArmElevator.setPower(1);
            _telemetry.addData("Grabber current position:", grabber_current_position);
            _telemetry.addData("Power set to: ", 1);
            //grabberArmElevator.setTargetPosition(grabber_max_position);
            //_telemetry.addData("Target Position Set To:", 700 );
        } else {
            _telemetry.addLine("Max Height Reached");
            grabberArmElevator.setPower(0);
        }
        _telemetry.addData("Grabber Max Possition",grabber_max_position);
    }

    public void ClawDown(){
        int startPosition;
        grabberArmElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabber_current_position = grabberArmElevator.getCurrentPosition();
        _telemetry.addData("Current Position:", grabber_current_position);
        if (grabber_current_position > grabber_min_position || !safety_net) {
            grabberArmElevator.setPower(-1);
            _telemetry.addData("Power set to: ", -1);
            _telemetry.addData("Grabber current position:", grabber_current_position);
            //grabberArmElevator.setTargetPosition(grabber_min_position);
            //_telemetry.addData("Target Position Set To:", 0);
        } else {
            _telemetry.addLine("Min Height Reached");
            grabberArmElevator.setPower(0);
        }
    }

    public void ClawOpen(){
        _telemetry.addLine("opening claw");
        servo1.setPosition(0.6);
    }

    public void ClawClosed(){
        servo1.setPosition(0);
        _telemetry.addLine("closing claw");


    }

    public DcMotor getGrabberArmElevator() {
        return grabberArmElevator;
    }
}





