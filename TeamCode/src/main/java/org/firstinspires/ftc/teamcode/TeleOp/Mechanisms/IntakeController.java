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
    private boolean powerEjecting = false;

    private double startTSEject;
    private final ElapsedTime bufferTime = new ElapsedTime();
    private static final int DELAY_MS_EJECT = 5000;

    private double startTSIntake;
    private static final int DELAY_MS_Intake = 5000;
    private boolean setBeforeIntake = false;
    private boolean isSetBeforeEject = false;
    private boolean delayIntake = false;
    private boolean delayEject = false;


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
                if (!setBeforeIntake) {
                    startTSIntake = bufferTime.milliseconds();
                    setBeforeIntake = true;
                }
                if (delayIntake()) {
                    intake.powerEject();
                    powerEjecting = true;
                }

            }
            if (powerEjecting) {
                if(!isSetBeforeEject) {
                    startTSEject = bufferTime.milliseconds();
                    isSetBeforeEject = true;
                }
                if (delayEject()) {
                    intake.intake();
                }
                if(!intake.intakeJammed()) {
                    rampUp = true;
                }

            }

            /* The following is the logic that uses the power eject velocity instead of time to which back to intake.
               We changed to time because the battery can effect the velocity. This does not allow the logic to work.
             */
            //if(intake.getIntakeVelocity() <= (powerEjectSetVelocity + powerEjectVelocityMargin)) {
              //  rampUp = true;
                //intake.intake();
           // }

        }

    }
    public boolean delayIntake() {
        double finalTS = bufferTime.milliseconds();
        delayIntake =  (finalTS - startTSIntake) >= DELAY_MS_Intake;
        return delayIntake;
    }
    public boolean delayEject() {
        double finalTS = bufferTime.milliseconds();
        delayEject = (finalTS - startTSEject) >= DELAY_MS_EJECT;
        return delayEject;
    }

    public boolean isSetBeforeIntake() {
        return setBeforeIntake;
    }

    public boolean isSetBeforeEject() {
        return isSetBeforeEject;
    }

    public boolean isDelayIntake() {
        return delayIntake;
    }

    public boolean isDelayEject() {
        return delayEject;
    }

    public double getStartTSEject() {
        return startTSEject;
    }

    public double getStartTSIntake() {
        return startTSIntake;
    }

    public boolean isRampUp() {
        return rampUp;
    }
}