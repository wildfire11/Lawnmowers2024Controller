package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class MoveArm {
    private LinearSlideElevator linearSlideElevator = null;
     public MoveArm(HardwareMap hardwareMap, Telemetry telemetry){
         linearSlideElevator = new LinearSlideElevator(hardwareMap, telemetry);
    }
    public class JustBelowTopBar implements Action {
        private boolean initialized = false;
        private LinearSlideElevator linearSlideElevator = null;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                linearSlideElevator.clawUp();
                initialized = true;
            }

            return false;
        }
    }
    public class AtTopBar implements Action {
        private boolean initialized = false;
        private LinearSlideElevator linearSlideElevator = null;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                linearSlideElevator.clawReadyToPull();
                initialized = true;
            }

            return false;
        }
    }

    public Action justBelowTopBar() {
        return new JustBelowTopBar();
    }

    public Action atTopBar() {
         return new AtTopBar();
    }

}



@Config
@Autonomous(name = "Simple Auton Teleop", group = "Autonomous")
public class SimpleStrafeAuton extends LinearOpMode {


    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        MoveArm moveArm = new MoveArm(hardwareMap, telemetry);
        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)

                .turn(90)
                .strafeTo(new Vector2d(-40,-30))
                .lineToY(-30);

        TrajectoryActionBuilder moveAfterArmUp = drive.actionBuilder(initialPose)
                .lineToY(-45);




        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose);
//                .lineToY(37)
//                .setTangent(Math.toRadians(0))
//                .lineToX(18)
//                .waitSeconds(3)
//                .setTangent(Math.toRadians(0))
//                .lineToXSplineHeading(46, Math.toRadians(180))
//                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose);
//                .lineToYSplineHeading(33, Math.toRadians(180))
//                .waitSeconds(2)
//                .strafeTo(new Vector2d(46, 30))
//                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.fresh()
//                .strafeTo(new Vector2d(48, 12))
                .build();



        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        trajectoryActionCloseOut,
                        moveArm.justBelowTopBar(),
                        moveAfterArmUp.build(),
                        moveArm.atTopBar()


                )
        );
    }
}

