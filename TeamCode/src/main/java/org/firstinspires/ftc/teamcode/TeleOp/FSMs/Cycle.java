package org.firstinspires.ftc.teamcode.TeleOp.FSMs;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Gripper;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.IntakeController;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Slides;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.TransferController;

public class Cycle {

    public enum CycleFSM {
        start,
        extend,
        retract,
        outtakeLeft,
        releaseRight,
        gripLeft,
        gripRight,
        posUp,
        posDown,
        offsetUp,
        offsetDown,
        stopIntake,
        releaseBoth,
        gripBoth
    }

    private CycleFSM state = CycleFSM.start;
    private final TransferController transferController;
    private final IntakeController intakeController;
    private final Gripper gripper;
    private final GamepadEx gamepadEx;
    private final Telemetry telemetry;
    private boolean toTransfer = false;


    private double prevLeftTrigger = 0.0;
    private double prevRightTrigger = 0.0;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;
    private boolean prevDpadRight = false;
    private boolean prevY = false;
    private boolean prevA = false;

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
        switch (state) {
            case start:
                checkInputs();
                break;
            case extend:
                toTransfer = true;
                if (transferController.extend())
                    state = CycleFSM.start;
                else
                    checkInputs();//Just to see if the driver wants to retract

                break;

            case retract:
                toTransfer = true;
                gripper.releaseRight();
                gripper.releaseLeft();
                if (transferController.retract()) {
                    toTransfer = false;
                    state = CycleFSM.start;
                    break;
                } else
                    checkInputs();//Just to see if the driver wants to extend
                break;
            case outtakeLeft:
                gripper.releaseLeft();
                toTransfer = true;
                state = CycleFSM.start;
                break;

            case releaseRight:
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

            case posUp:
                transferController.pos_up();
                state = CycleFSM.start;
                break;

            case posDown:
                transferController.pos_down();
                state = CycleFSM.start;
                break;

            case offsetUp:
                transferController.offset_up();
                state = CycleFSM.start;
                break;

            case offsetDown:
                transferController.offset_down();
                state = CycleFSM.start;
                break;
            case stopIntake:
                intakeController.setStopRequested(!intakeController.getStopRequested());
                state = CycleFSM.start;
                break;
            case releaseBoth:
                gripper.releaseLeft();
                gripper.releaseRight();
                state = CycleFSM.start;
                break;
            case gripBoth:
                gripper.gripLeft();
                gripper.gripRight();
                state = CycleFSM.start;
                break;
        }

    }

    public void checkInputs() {
        //Index Up
        if (gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 1.0 & prevLeftTrigger != 1.0) {
            transferController.pos_up();
        }
        //Index down
        if (gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 1.0 & prevRightTrigger != 1.0) {
            transferController.pos_down();
        }
        //Offset up
        if (gamepadEx.isDown(GamepadKeys.Button.DPAD_UP) & !prevDpadUp) {
            transferController.offset_up();
        }
        //Offset down
        if (gamepadEx.isDown(GamepadKeys.Button.DPAD_DOWN) & !prevDpadDown) {
            transferController.offset_down();
        }
        //Extend
        if (gamepadEx.isDown(GamepadKeys.Button.Y) & !prevY)
            state = CycleFSM.extend;
        //Retract
        if (gamepadEx.isDown(GamepadKeys.Button.A) & !prevA)
            state = CycleFSM.retract;
        //Gripping and ungripping pixels on the right claw.
        if (gamepadEx.isDown(GamepadKeys.Button.RIGHT_BUMPER) & !prevRightBumper) {
            if (gripper.getRightClawGripped())
                state = CycleFSM.releaseRight;
            else
                state = CycleFSM.gripRight;
        }
        //Gripping and ungripping pixels on the left claw.
        if (gamepadEx.isDown(GamepadKeys.Button.LEFT_BUMPER) & !prevLeftBumper) {
            if (gripper.getLeftClawGripped())
                state = CycleFSM.outtakeLeft;
            else
                state = CycleFSM.gripLeft;
        }
        //Stopping the intake
        if (gamepadEx.isDown(GamepadKeys.Button.DPAD_RIGHT) & !prevDpadRight) {
            state = CycleFSM.stopIntake;
        }
        //Release and grip pixels at the same time
        if (gamepadEx.isDown(GamepadKeys.Button.RIGHT_BUMPER) & !prevRightBumper & gamepadEx.isDown(GamepadKeys.Button.LEFT_BUMPER) & !prevLeftBumper) {
            if (gripper.getLeftClawGripped() && gripper.getRightClawGripped())
                state = CycleFSM.releaseBoth;
            else
                state = CycleFSM.gripBoth;
        }

        prevLeftTrigger = gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        prevRightTrigger = gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        prevDpadUp = gamepadEx.isDown(GamepadKeys.Button.DPAD_UP);
        prevDpadDown = gamepadEx.isDown(GamepadKeys.Button.DPAD_DOWN);
        prevY = gamepadEx.isDown(GamepadKeys.Button.Y);
        prevA = gamepadEx.isDown(GamepadKeys.Button.A);
        prevRightBumper = gamepadEx.isDown(GamepadKeys.Button.RIGHT_BUMPER);
        prevLeftBumper = gamepadEx.isDown(GamepadKeys.Button.LEFT_BUMPER);
        prevDpadRight = gamepadEx.isDown(GamepadKeys.Button.DPAD_RIGHT);

    }

    public boolean getToTransfer() {
        return toTransfer;
    }


}
