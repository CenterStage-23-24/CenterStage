package org.firstinspires.ftc.teamcode.TeleOp.FSMs;

import android.widget.Button;

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
        outtakeBoth,
        pos_up,
        pos_down,
        offset_up,
        offset_down
    }

    private CycleFSM state = CycleFSM.start;
    private final FSMController fsmController;
    private final TransferController transferController;

    private boolean toTransfer = false;


    private final GamepadEx gamepad;
    private final Telemetry telemetry;

    private final Gripper gripper;
    private double prev_left_trigger = 0.0;
    private double prev_right_trigger = 0.0;
    private boolean prev_dpad_up = false;
    private boolean prev_dpad_down = false;

    public Cycle(GamepadEx gamepad, Telemetry telemetry, TransferController transferController, Gripper gripper) {
        //Core
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        //Controllers
        this.gripper = gripper;
        this.transferController = transferController;
        this.fsmController = new FSMController(gamepad);

        telemetry.addData("INIT: ", "Cycle");
        telemetry.update();
    }


    public void loop() {
        gamepad.readButtons();
        fsmController.readControllerInputs();
        switch (state) {
            case start:
                checkIndexInputs();
                //Outtake Left
                telemetry.addData("in start in cycle", 1);
                if (fsmController.getLeftBumper()) {
                    telemetry.addData("Left-Bumper pressed in cycle", 1);
                    state = CycleFSM.outtakeLeft;
                }

                //Outtake Right
                if (fsmController.getRightBumper()) {
                    telemetry.addData("Left-Bumper in cycle", 1);
                    state = CycleFSM.outtakeRight;
                }

                //Outtake Both
                if (fsmController.getRightBumper() && fsmController.getLeftBumper()) {
                    telemetry.addData("Left-Bumper in cycle", 1);
                    state = CycleFSM.outtakeBoth;
                }

                //Extend
                if (fsmController.getYButton()) {
                    telemetry.addData("y pressed in cycle", 1);
                    state = CycleFSM.extend;
                }

                //Retract
                if (fsmController.getAButton()) {
                    telemetry.addData("b pressed in cycle", 1);
                    state = CycleFSM.retract;
                }
                break;

            case extend:
                fsmController.setYButton(false);
                toTransfer = true;
                checkIndexInputs();
                if (transferController.extend()) {
                    state = CycleFSM.start;
                }
                break;

            case retract:
                fsmController.setAButton(false);
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
                fsmController.setLeftBumper(false);
                telemetry.addData("-", "Ready to deposit left");
                gripper.releaseLeft();
                toTransfer = true;
                state = CycleFSM.start;
                break;

            case outtakeRight:
                fsmController.setRightBumper(false);
                telemetry.addData("-", "Ready to deposit right");
                gripper.releaseRight();
                toTransfer = true;
                state = CycleFSM.start;
                break;

            case outtakeBoth:
                fsmController.setRightBumper(false);
                fsmController.setLeftBumper(false);
                telemetry.addData("-", "Ready to deposit right");
                gripper.releaseRight();
                gripper.releaseLeft();
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
        if(gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 1.0 & prev_left_trigger != 1.0){
            transferController.pos_up();
        }
        if(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 1.0 & prev_right_trigger != 1.0){
            transferController.pos_down();
        }
        if(gamepad.isDown(GamepadKeys.Button.DPAD_UP) & !prev_dpad_up){
            transferController.offset_up();
        }
        if(gamepad.isDown(GamepadKeys.Button.DPAD_DOWN) & !prev_dpad_down){
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

    public boolean getPrevUp(){return prev_dpad_up;}

    public boolean getPrevDown(){return prev_dpad_down;}
}


