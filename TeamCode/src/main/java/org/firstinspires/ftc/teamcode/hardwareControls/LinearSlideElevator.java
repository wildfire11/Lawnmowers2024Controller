package org.firstinspires.ftc.teamcode.hardwareControls;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LinearSlideElevator {
    private final DcMotor grabberArmElevator;
    private final Servo servo1;
    private final Telemetry telemetry;

    private Action currentAction = null;
    private boolean continuousMovement = false;
    public boolean safety_net = true;

    //Movement limits
    public static int grabberMaxPosition = 3800;
    public static int grabberMinPosition = 0;
    public static int clawReadyToPullHeight = 2800;
    public static int clawJustUnderTopBarHeight = 2665;

    public LinearSlideElevator(HardwareMap hardwareMap, Telemetry telemetry) {
        grabberArmElevator = hardwareMap.get(DcMotor.class, "grabberArmElevator");
        grabberArmElevator.setDirection(DcMotorSimple.Direction.FORWARD);
        grabberArmElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo1 = hardwareMap.get(Servo.class, "servo");
        this.telemetry = telemetry;
    }

    // Centralized logic for elevator movement
    private void moveElevator(int targetPosition, double power) {
        grabberArmElevator.setTargetPosition(targetPosition);
        grabberArmElevator.setPower(power);
        grabberArmElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Create an Action wrapper for any logic
    private Action createAction(Runnable logic) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    logic.run();
                    initialized = true;
                }
                return false; // Complete after one execution
            }
        };
    }

    public Action sequence(Action... actions) {
        return new Action() {
            private int currentIndex = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (currentIndex < actions.length && !actions[currentIndex].run(packet)) {
                    currentIndex++;
                }
                return currentIndex < actions.length;
            }
        };
    }

    // Elevator actions
    public Action clawToFloorAction() {
        return createAction(() -> moveElevator(0, 1));
    }

    public Action clawJustUnderTopBarAction() {
        telemetry.addLine("Just under top bar");
        return createAction(() -> moveElevator(clawJustUnderTopBarHeight, 1));
    }

    public Action clawReadyToPullAction() {
        return createAction(() -> moveElevator(clawReadyToPullHeight, 1));
    }

    public Action stopClawAction() {
        return createAction(() -> grabberArmElevator.setPower(0));
    }

    public Action clawOpenAction() {
        return createAction(() -> {
            servo1.setPosition(0.6);
            telemetry.addLine("Claw opened");
        });
    }

    public Action clawCloseAction() {
        return createAction(() -> {
            servo1.setPosition(0);
            telemetry.addLine("Claw closed");
        });
    }

    // Execute an Action incrementally
    public void executeAction(Action action) {
        this.currentAction = action;
    }

    // Call this in the main loop to process the current action
    public void processAction() {
        if (currentAction != null) {
            TelemetryPacket packet = new TelemetryPacket();
            if (!currentAction.run(packet)) {
                // Action is complete
                currentAction = null;
            }
        }
    }

    // Continuous movement logic
    public void moveClawContinuously(double power) {
        continuousMovement = true;
        grabberArmElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabberArmElevator.setPower(power);
    }

    public void stopContinuousMovement() {
        continuousMovement = false;
        grabberArmElevator.setPower(0);
    }

    // Teleop logic
    public void handleTeleop(boolean moveUpButtonPressed, boolean moveDownButtonPressed) {
        if (currentAction != null) {
            // If an action is in progress, let it complete
            TelemetryPacket packet = new TelemetryPacket();
            if (!currentAction.run(packet)) {
                currentAction = null; // Action finished
            }
        } else if (moveUpButtonPressed) {
            // Handle continuous movement
            moveClawContinuously(1);
        } else if (moveDownButtonPressed) {
            // Handle continuous movement
            moveClawContinuously(-1);
        }
        else if (continuousMovement) {
            // Stop continuous movement if button is released
            stopContinuousMovement();
        }
    }
}
