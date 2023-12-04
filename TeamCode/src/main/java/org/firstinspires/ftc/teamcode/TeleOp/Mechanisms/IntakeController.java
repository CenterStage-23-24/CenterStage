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
    private final int powerEjectSetVelocity = -1500;
    private final int powerEjectVelocityMargin = 300;
    private boolean rampUp = true;
    private boolean powerEjecting = false;


    public IntakeController(Intake intake, GamepadEx gamepad, Gripper gripper) {
        this.intake = intake;
        this.gamepad = gamepad;
        this.gripper = gripper;
    }

    public boolean isPowerEjecting() {
        return powerEjecting;
    }

    public void intakeControl(boolean toTransfer) {
        intakeRunning = false;
        powerEjecting = false;
        if(intake.getIntakeVelocity() > 2000) { // edge case exists(has not happened yet) that if the the velocity doesn't go over 2000 due to battery
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

            if (intake.intakeJammed()) {
                    intake.powerEject();
                    powerEjecting = true;
                if(intake.getIntakeVelocity() <= (powerEjectSetVelocity + powerEjectVelocityMargin)) { // if doesn't work could be issue with battery as velocity is not reaching high enough. Could try to lower the velocity needed to be reached or using wait time.
                      rampUp = true; // should work based on testing on Sunday. Make sure it does though
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
}