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
        gripper
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
                //Extend
                if (gamepad.isDown(GamepadKeys.Button.Y)) {
                    telemetry.addData("y pressed in cycle", 1);
                    state = CycleFSM.extend;
                }

            case extend:
                toTransfer = true;
                if (transferController.extend()) {
                    state = CycleFSM.gripper;
                }
                break;

            case gripper:
                while (!(gamepad.isDown(GamepadKeys.Button.A))) {
                    if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                        telemetry.addData("Left-Bumper pressed in cycle", 1);
                    }
                    if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                        telemetry.addData("Left-Bumper in cycle", 1);
                    }
                }

                state = CycleFSM.retract;
                break;

            case retract:
                toTransfer = true;
                gripper.releaseRight();
                gripper.releaseLeft();
                if (transferController.retract()) {
                    state = CycleFSM.start;
                    toTransfer = false;
                }
                break;
        }
    }
    public boolean getToTransfer() {
        return toTransfer;
    }
}


