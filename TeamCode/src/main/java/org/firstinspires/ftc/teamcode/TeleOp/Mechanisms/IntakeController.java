package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class IntakeController {

    private final Intake intake;
    private final Gripper gripper;
    private final GamepadEx gamepad;
    private boolean stopRequested = false;
    private boolean intakeRunning= false;

    public IntakeController(Intake intake, GamepadEx gamepad, Gripper gripper) {
        this.intake = intake;
        this.gamepad = gamepad;
        this.gripper = gripper;
    }

    public void intakeControl(boolean toTransfer) {
        intakeRunning = false;
        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            stopRequested = !stopRequested;

        if (!stopRequested) {

            if (!toTransfer) {
                intake.detectPixels();
                if (intake.getPixelInLeft())
                    gripper.gripLeft();
                if (intake.getPixelInRight())
                    gripper.gripRight();

                if ((!intake.getPixelInLeft() || !intake.getPixelInRight())) {
                    intake.intake();
                    intakeRunning = true;
                }
                else
                    intake.eject();
            } else
                intake.eject();
        } else
            intake.intake(0);

        if (intakeRunning) {
            if(intake.intakeJammed()) {
                intake.powerEject();
            }

        }

    }
}