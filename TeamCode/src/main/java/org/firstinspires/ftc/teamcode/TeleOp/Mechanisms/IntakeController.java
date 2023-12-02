package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;


public class IntakeController {

    private final Intake intake;
    private final Gripper gripper;
    private final GamepadEx gamepad;
    private boolean stopRequested = false;
    private boolean intakeRunning= false;
    private final int powerEjectSetVelocity = -1600;
    private final int powerEjectVelocityMargin = 100;
    private boolean rampUp = true;


    public IntakeController(Intake intake, GamepadEx gamepad, Gripper gripper) {
        this.intake = intake;
        this.gamepad = gamepad;
        this.gripper = gripper;
    }

    public void intakeControl(boolean toTransfer) {
        intakeRunning = false;
        if(intake.getIntakeVelocity() > 2000) {
            rampUp = false;
        }
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
                    intakeRunning = true;
                    intake.intake();
                }
                else {
                    intake.eject();
                    rampUp = true;
                }
            } else {
                intake.eject();
                rampUp = true;
            }
        } else {
            intake.intake(0);
            rampUp = true;
        }

        if (intakeRunning && !rampUp) {
            if(intake.intakeJammed()) {
                intake.powerEject();
                rampUp = true; // change to work
            }
            if(intake.getIntakeVelocity() <= (powerEjectSetVelocity + powerEjectVelocityMargin)) {
                intake.intake();
            }

        }

    }
}