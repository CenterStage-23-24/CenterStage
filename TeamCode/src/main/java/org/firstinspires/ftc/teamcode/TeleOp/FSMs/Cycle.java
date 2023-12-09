package org.firstinspires.ftc.teamcode.TeleOp.FSMs;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Gripper;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.TransferController;

public class Cycle {

    public enum CycleFSM {
        start,
        extend,
        retract,
        outtakeLeft,
        outtakeRight,
        outtakeBoth
    }

    private CycleFSM state = CycleFSM.start;
    private final FSMController fsmController;
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
        this.fsmController = new FSMController(gamepad);

        telemetry.addData("INIT: ", "Cycle");
        telemetry.update();
    }


    public void loop() {
        gamepad.readButtons();
        fsmController.readControllerInputs();
        switch (state) {
            case start:
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
                if (transferController.extend()) {
                    state = CycleFSM.start;
                }
                break;

            case retract:
                fsmController.setAButton(false);
                toTransfer = true;
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

        }
    }
    public boolean getToTransfer() {
        return toTransfer;
    }
}
