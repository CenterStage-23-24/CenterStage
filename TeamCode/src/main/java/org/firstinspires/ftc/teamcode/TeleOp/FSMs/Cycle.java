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
        stop_intake,
        outtake_both,
        grip_both
    }

    private CycleFSM state = CycleFSM.start;
    private final TransferController transferController;
    private final IntakeController intakeController;

    private boolean toTransfer = false;


    private final GamepadEx gamepadEx;
    private final Telemetry telemetry;

    private final Gripper gripper;
    private double prev_left_trigger = 0.0;
    private double prev_right_trigger = 0.0;
    private boolean prev_dpad_up = false;
    private boolean prev_dpad_down = false;
    private boolean prev_right_bumper = false;
    private boolean prev_left_bumper = false;
    private boolean prev_dpad_right = false;


    public Cycle(GamepadEx gamepadEx, Telemetry telemetry, TransferController transferController, IntakeController intakeController, Gripper gripper) {
        //Core
        this.gamepadEx = gamepadEx;
        this.telemetry = telemetry;
        //Controllers
        this.gripper = gripper;
        this.transferController = transferController;
        this.intakeController = intakeController;
        telemetry.addData("INIT: ", "Cycle");
        telemetry.update();
    }


    public void loop() {
        checkInputs();
        switch (state) {
            case start:
                checkInputs();
                //Outtake Left
                telemetry.addData("in start in cycle", 1);
                //Extend
                if (gamepadEx.isDown(GamepadKeys.Button.Y)) {
                    state = CycleFSM.extend;
                }

                //Retract
                if (gamepadEx.isDown(GamepadKeys.Button.A)) {
                    state = CycleFSM.retract;
                }
                break;

            case extend:
                toTransfer = true;
                checkInputs();
                if (transferController.extend()) {
                    state = CycleFSM.start;
                }

                break;

            case retract:
                toTransfer = true;
                checkInputs();
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
            case outtake_both:
                gripper.releaseLeft();
                gripper.releaseRight();
                state = CycleFSM.start;
            case grip_both:
                gripper.gripLeft();
                gripper.gripRight();
                state = CycleFSM.start;
        }

    }

    public void checkInputs() {
        if (gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 1.0 & prev_left_trigger != 1.0) {
            transferController.pos_up();
        }
        if (gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 1.0 & prev_right_trigger != 1.0) {
            transferController.pos_down();
        }
        if (gamepadEx.isDown(GamepadKeys.Button.DPAD_UP) & !prev_dpad_up) {
            transferController.offset_up();
        }
        if (gamepadEx.isDown(GamepadKeys.Button.DPAD_DOWN) & !prev_dpad_down) {
            transferController.offset_down();
        }
        if (gamepadEx.isDown(GamepadKeys.Button.RIGHT_BUMPER) & !prev_right_bumper) {
            if (gripper.getRightClawGripped())
                state = CycleFSM.outtakeRight;
            else
                state = CycleFSM.gripRight;
        }
        if (gamepadEx.isDown(GamepadKeys.Button.LEFT_BUMPER) & !prev_left_bumper) {
            if (gripper.getLeftClawGripped())
                state = CycleFSM.outtakeLeft;
            else
                state = CycleFSM.gripLeft;
        }
        if (gamepadEx.isDown(GamepadKeys.Button.DPAD_RIGHT) & !prev_dpad_right) {
            state = CycleFSM.stop_intake;
        }
        if (gamepadEx.isDown(GamepadKeys.Button.RIGHT_BUMPER) & !prev_right_bumper & gamepadEx.isDown(GamepadKeys.Button.LEFT_BUMPER) & !prev_left_bumper) {
            if (gripper.getLeftClawGripped() && gripper.getRightClawGripped())
                state = CycleFSM.outtake_both;
            else
                state = CycleFSM.grip_both;
        }
        prev_left_trigger = gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        prev_right_trigger = gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        prev_dpad_up = gamepadEx.isDown(GamepadKeys.Button.DPAD_UP);
        prev_dpad_down = gamepadEx.isDown(GamepadKeys.Button.DPAD_DOWN);
        prev_right_bumper = gamepadEx.isDown(GamepadKeys.Button.RIGHT_BUMPER);
        prev_left_bumper = gamepadEx.isDown(GamepadKeys.Button.LEFT_BUMPER);
        prev_dpad_right = gamepadEx.isDown(GamepadKeys.Button.DPAD_RIGHT);

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
