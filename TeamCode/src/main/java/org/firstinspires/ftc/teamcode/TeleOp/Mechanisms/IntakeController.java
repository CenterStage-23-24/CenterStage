package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class IntakeController {

    private final Intake intake;
    private final Gripper gripper;
    private final GamepadEx gamepad;
    private boolean stopRequested = false;
    private boolean intakeRunning= false;
    public static int POWER_EJECT_SET_VELOCITY = -1200; // public static right now
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
        if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            jammingDisabled = !jammingDisabled;
        }
        intakeRunning = false;
        powerEjecting = false;
        if(intake.getIntakeVelocity() > 1500) { // edge case exists(has not happened yet) that if the the velocity doesn't go over 2000 due to battery
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

        if (intakeRunning && !rampUp && !jammingDisabled) {

            if (intake.intakeJammed()) {
                    intake.powerEject();
                    powerEjecting = true;
                if(intake.getIntakeVelocity() <= (POWER_EJECT_SET_VELOCITY)) { // if doesn't work could be issue with battery as velocity is not reaching high enough. Could try to lower the velocity needed to be reached or using wait time.
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