package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class IntakeController {

    private final Intake intake;
    private final GamepadEx gamepad;
    private boolean stopRequested = false;

    public IntakeController(Intake intake, GamepadEx gamepad) {
        this.intake = intake;
        this.gamepad = gamepad;
    }

    public void intakeControl(boolean toTransfer) {
        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            stopRequested = !stopRequested;

        if (!stopRequested) {

            if (!toTransfer) {
                intake.detectPixels();
                if (intake.getPixelInLeft())
                    intake.gripLeft();
                if (intake.getPixelInRight())
                    intake.gripRight();
                if ((!intake.getPixelInLeft() || !intake.getPixelInRight())) {
                    intake.intake();
                } else {
                    intake.eject();
                }
            } else {
                intake.eject();
            }
        } else {
            intake.intake(0);
        }
    }
}