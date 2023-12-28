package org.firstinspires.ftc.teamcode.TeleOp.FSMs;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Gripper;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.IntakeController;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.TransferController;

public class Cycle {

    public enum CycleFSM {
        start,
        extend,
        retract,
        outtakeLeft,
        outtakeRight,
        gripLeft,
        gripRight,
        pos_up,
        pos_down,
        offset_up,
        offset_down,
        stop_intake
    }

    private CycleFSM state = CycleFSM.start;
    private final TransferController transferController;
    private final IntakeController intakeController;

    private boolean toTransfer = false;


    private final GamepadEx gamepad;
    private final Telemetry telemetry;

    private final Gripper gripper;
    private double prev_left_trigger = 0.0;
    private double prev_right_trigger = 0.0;
    private boolean prev_dpad_up = false;
    private boolean prev_dpad_down = false;


    public Cycle(GamepadEx gamepad, Telemetry telemetry, TransferController transferController, IntakeController intakeController, Gripper gripper) {
        //Core
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        //Controllers
        this.gripper = gripper;
        this.transferController = transferController;
        this.intakeController = intakeController;
        telemetry.addData("INIT: ", "Cycle");
        telemetry.update();
    }


    public void loop() {
        gamepad.readButtons();
        switch (state) {
            case start:
                checkIndexInputs();
                //Outtake Left
                telemetry.addData("in start in cycle", 1);
                if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    telemetry.addData("Left-Bumper pressed in cycle", 1);
                    if (gripper.getLeftClawGripped())
                        state = CycleFSM.outtakeLeft;
                    else
                        state = CycleFSM.gripLeft;
                }

                //Outtake Right
                if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    telemetry.addData("Left-Bumper in cycle", 1);
                    if (gripper.getRightClawGripped())
                        state = CycleFSM.outtakeRight;
                    else
                        state = CycleFSM.gripRight;
                }

                //Extend
                if (gamepad.isDown(GamepadKeys.Button.Y)) {
                    telemetry.addData("y pressed in cycle", 1);
                    state = CycleFSM.extend;
                }

                //Retract
                if (gamepad.isDown(GamepadKeys.Button.A)) {
                    telemetry.addData("b pressed in cycle", 1);
                    state = CycleFSM.retract;
                }
                if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
                    state = CycleFSM.stop_intake;
                break;

            case extend:
                toTransfer = true;
                checkIndexInputs();
                if (transferController.extend()) {
                    state = CycleFSM.start;
                }
                break;

            case retract:
                toTransfer = true;
                checkIndexInputs();
                gripper.releaseRight();
                gripper.releaseLeft();
                if (transferController.retract()) {
                    state = CycleFSM.start;
                    toTransfer = false;
                    break;
                }
                break;
            case outtakeLeft:
                gripper.releaseLeft();
                toTransfer = true;
                state = CycleFSM.start;
                break;

            case outtakeRight:
                gripper.releaseRight();
                toTransfer = true;
                state = CycleFSM.start;
                break;
            case gripLeft:
                gripper.gripLeft();
                toTransfer = true;
                state = CycleFSM.start;
                break;

            case gripRight:
                gripper.gripRight();
                toTransfer = true;
                state = CycleFSM.start;
                break;

            case pos_up:
                transferController.pos_up();
                state = CycleFSM.start;
                break;

            case pos_down:
                transferController.pos_down();
                state = CycleFSM.start;
                break;

            case offset_up:
                transferController.offset_up();
                state = CycleFSM.start;
                break;

            case offset_down:
                transferController.offset_down();
                state = CycleFSM.start;
                break;
            case stop_intake:
                intakeController.setStopRequested(!intakeController.getStopRequested());
                state = CycleFSM.start;
                break;

        }

    }

    public void checkIndexInputs() {
        if (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 1.0 & prev_left_trigger != 1.0) {
            transferController.pos_up();
        }
        if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 1.0 & prev_right_trigger != 1.0) {
            transferController.pos_down();
        }
        if (gamepad.isDown(GamepadKeys.Button.DPAD_UP) & !prev_dpad_up) {
            transferController.offset_up();
        }
        if (gamepad.isDown(GamepadKeys.Button.DPAD_DOWN) & !prev_dpad_down) {
            transferController.offset_down();
        }

        prev_left_trigger = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        prev_right_trigger = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        prev_dpad_up = gamepad.isDown(GamepadKeys.Button.DPAD_UP);
        prev_dpad_down = gamepad.isDown(GamepadKeys.Button.DPAD_DOWN);
    }

    public boolean getToTransfer() {
        return toTransfer;
    }

    public boolean getPrevUp() {
        return prev_dpad_up;
    }

    public boolean getPrevDown() {
        return prev_dpad_down;
    }


}


