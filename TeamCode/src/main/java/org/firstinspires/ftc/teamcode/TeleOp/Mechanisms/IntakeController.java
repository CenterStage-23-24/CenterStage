package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class IntakeController {

    private final Intake intake;
    private final Gripper gripper;
    private final GamepadEx gamepad;
    private boolean stopRequested = false;

    private boolean intakeRunning = false;
    private final static int POWER_EJECT_SET_VELOCITY = -400;
    private boolean rampUp = true;
    private boolean powerEjecting = false;
    private boolean jammingDisabled = false;


    public IntakeController(Intake intake, GamepadEx gamepad, Gripper gripper) {
        this.intake = intake;
        this.gamepad = gamepad;
        this.gripper = gripper;
    }

    public boolean isPowerEjecting() {
        return powerEjecting;
    }

    public void intakeControl(boolean toTransfer) {
        gamepad.readButtons();
        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            jammingDisabled = !jammingDisabled;
        }
        intakeRunning = false;
        powerEjecting = false;
        if (intake.getIntakeVelocity() > 1500) {
            rampUp = false;
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
            stopRequested = !stopRequested;

        if (!stopRequested) {
            if(!gamepad.isDown(GamepadKeys.Button.B)) {
                if (!toTransfer) {
                    intake.detectPixels();
                    if (intake.getPixelInLeft()) {
                        gripper.gripLeft();
                    }
                    if (intake.getPixelInRight()) {
                        gripper.gripRight();
                    }
                    if ((!intake.getPixelInLeft() || !intake.getPixelInRight())) {
                        intake.intake();
                    } else {
                        intake.eject();
                        rampUp = true;
                    }
                } else {
                    intake.eject();
                    rampUp = true;
                }
            } else{
                intake.eject();
                rampUp = true;
            }
        } else {
            intake.intake(0);
            rampUp = true;
        }

        if (intakeRunning && !rampUp && !jammingDisabled) {

            if (intake.intakeJammed()) {
                intake.powerEject();
                powerEjecting = true;
                if (intake.getIntakeVelocity() <= (POWER_EJECT_SET_VELOCITY)) {
                    rampUp = true;
                    intake.intake();
                }


            }

        }


    }

    public boolean isRampUp() {
        return rampUp;
    }

    public boolean isIntakeRunning() {
        return intakeRunning;
    }

    public boolean isJammingDisabled() {
        return jammingDisabled;
    }


}