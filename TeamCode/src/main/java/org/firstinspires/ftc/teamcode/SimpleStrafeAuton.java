package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardwareControls.LinearSlideElevator;


@Config
@Autonomous(name = "Simple Auton Teleop", group = "Autonomous")
public class SimpleStrafeAuton extends LinearOpMode {


    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        LinearSlideElevator linearSlideActions = new LinearSlideElevator(hardwareMap, telemetry);
        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder driveToBar = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-40,-30));




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
                        driveToBar.build(),
                        linearSlideActions.clawJustUnderTopBarAction()
                        //trajectoryActionCloseOut,
                        //moveArm.justBelowTopBar(),
                        //moveAfterArmUp.build(),
                        //moveArm.atTopBar()


                )
        );
    }
}

