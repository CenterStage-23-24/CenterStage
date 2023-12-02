package org.firstinspires.ftc.teamcode.TeleOp.FSMs;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Gripper;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Slides;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.TransferController;

public class Cycle {

    public enum CycleFSM {
        start,
        extend,
        retract,
        outtakeLeft,
        outtakeRight,
        pos_up,
        pos_down,
        offset_up,
        offset_down
    }

    private CycleFSM state = CycleFSM.start;
    private final TransferController transferController;

    private boolean toTransfer = false;


    private final GamepadEx gamepad;
    private final Telemetry telemetry;

    private final Gripper gripper;

    public Cycle(GamepadEx gamepad, Telemetry telemetry, TransferController transferController, Gripper gripper) {
        //Core
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        //Controllers
        this.gripper = gripper;
        this.transferController = transferController;

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
                    state = CycleFSM.outtakeLeft;
                }

                //Outtake Right
                if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    telemetry.addData("Left-Bumper in cycle", 1);
                    state = CycleFSM.outtakeRight;
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

                //Offset up
                if(gamepad.isDown(GamepadKeys.Button.DPAD_UP)){
                    telemetry.addData("dpad_up pressed in cycle", 1);
                    state = CycleFSM.offset_up;
                }

                //Offset down
                if(gamepad.isDown(GamepadKeys.Button.DPAD_DOWN)){
                    telemetry.addData("dpad_down pressed in cycle", 1);
                    state = CycleFSM.offset_down;
                }
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
                }
                break;

            case outtakeLeft:
                telemetry.addData("-", "Ready to deposit left");
                gripper.releaseLeft();
                toTransfer = true;
                state = CycleFSM.start;
                break;

            case outtakeRight:
                telemetry.addData("-", "Ready to deposit right");
                gripper.releaseRight();
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

        }

    }

    public void checkIndexInputs(){
        if(gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 1.0){
            state = CycleFSM.pos_up;
        }
        if(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 1.0){
            state = CycleFSM.pos_down;
        }
    }

    public boolean getToTransfer() {
        return toTransfer;
    }
}


