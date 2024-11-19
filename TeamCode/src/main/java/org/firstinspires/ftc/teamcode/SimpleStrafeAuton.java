package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Config
@Autonomous(name = "Simple Auton Teleop", group = "Autonomous")
public class SimpleStrafeAuton extends LinearOpMode {

    private double unitsPerInch =73.4/45 ;
    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(16, -61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder driveToBar = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(70,-55.7));
        // .turn(Math.toRadians(90))
        //.strafeTo(new Vector2d(-9,-30))
        ;




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

        Actions.runBlocking(
                new SequentialAction(
                        driveToBar.build()
                        //trajectoryActionCloseOut,
                        //moveArm.justBelowTopBar(),
                        //moveAfterArmUp.build(),
                        //moveArm.atTopBar()


                )
        );

    }
}

