package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Cycle {

    public enum cycleFSM {
        start,
        intake,
        ejection,
        transfer,
        outtake,
        outtakeReverse
    }

    cycleFSM state = cycleFSM.start;

    GamepadEx gamepad;
    Telemetry telemetry;

    public Cycle (GamepadEx gamepad, Telemetry telemetry) {
        this.gamepad = gamepad; // add control class to program
        this.telemetry = telemetry;
    }


    public void loop() {
        while(true) {
            gamepad.readButtons();
            telemetry.addData("In cycle", 1);
            telemetry.addData("in while loop in cycle", 1);
            switch (state) {
                case start:
                    telemetry.addData("in start in cycle", 1);
                    if (gamepad.getButton(GamepadKeys.Button.A)) {
                        telemetry.addData("a pressed in cycle", 1);
                        state = cycleFSM.intake;
                    }
                    if (gamepad.getButton(GamepadKeys.Button.B)) {
                        telemetry.addData("b pressed in cycle", 1);
                        state = cycleFSM.ejection;
                    }
                    if (gamepad.getButton(GamepadKeys.Button.Y)) {
                        telemetry.addData("y pressed in cycle", 1);
                        state = cycleFSM.transfer;
                    }
                    if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                        telemetry.addData("left_bumper pressed in cycle", 1);
                        state = cycleFSM.outtake;
                    }
                    if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                        state = cycleFSM.outtakeReverse;
                    }
                    break;
                case intake:
                    telemetry.addData("In intake", 1);
                    state = cycleFSM.start;
                    break;
                case ejection:
                    telemetry.addData("In ejection", 1);
                    state = cycleFSM.start;
                    break;
                case transfer:
                    telemetry.addData("In transfer", 1);
                    state = cycleFSM.start;
                    break;
                case outtake:
                    telemetry.addData("In outtake", 1);
                    state = cycleFSM.start;
                    break;
                case outtakeReverse:
                    telemetry.addData("In outtake reverse", 1);
                    state = cycleFSM.start;
                    return;

            }
            telemetry.update();

            //drivetrain code here
            gamepad.readButtons();

        }
    }
}
