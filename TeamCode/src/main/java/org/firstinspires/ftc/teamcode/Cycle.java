package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

public class Cycle {

    public enum cycleFSM {
        start,
        intake,
        ejection,
        transfer,
        outtake
    }

    cycleFSM state = cycleFSM.start;
    private DcMotorEx intakeMotor;
    private DcMotorEx linearSlidesRight;
    private DcMotorEx linearSlidesLeft;
    private Servo outakeServoLeft;
    private Servo outakeServoRight;
    Gamepad gamepad;
    Telemetry telemetry;

    public Cycle (HWMap hardware, Gamepad gamepad, Telemetry telemetry) {
        intakeMotor = hardware.getIntakeMotor();
        linearSlidesRight = hardware.getLinearSlidesRight();
        linearSlidesLeft = hardware.getLinearSlidesLeft();
        outakeServoLeft = hardware.getOutakeServoLeft();
        outakeServoRight = hardware.getOutakeServoRight();
        this.gamepad = gamepad; // add control class to program
        this.telemetry = telemetry;

    }


    public void loop() {
        while(true) {
            telemetry.addData("In cycle", 1);
            telemetry.addData("in while loop in cycle", 1);
            switch (state) {
                case start:
                    telemetry.addData("in start in cycle", 1);
                    if (gamepad.a) {
                        telemetry.addData("a pressed in cycle", 1);
                        state = cycleFSM.intake;
                    }
                    /*
                    if (gamepad.options) {
                        telemetry.addData("options pressed in cycle", 1);
                        state = cycleFSM.ejection;
                    }
                    if (gamepad.y) {
                        telemetry.addData("y pressed in cycle", 1);
                        state = cycleFSM.transfer;
                    }
                    if (gamepad.left_bumper) {
                        telemetry.addData("left_bumper pressed in cycle", 1);
                        state = cycleFSM.outtake;
                    }
                     */
                    break;
                case intake:
                    telemetry.addData("In intake", 1);
                    intakeMotor.setPower(0.5);
                    state = cycleFSM.start;
                    break;
                    /*
                case ejection:
                    telemetry.addData("In ejection", 1);
                    intakeMotor.setPower(-0.5);
                    state = cycleFSM.start;
                    break;
                case transfer:
                    telemetry.addData("In transfer", 1);
                    linearSlidesLeft.setPower(0.5);
                    linearSlidesRight.setPower(0.5);
                    state = cycleFSM.start;
                    break;
                case outtake:
                    telemetry.addData("In outtake", 1);
                    outakeServoLeft.setPosition(0.75);
                    outakeServoRight.setPosition(0.75);
                    state = cycleFSM.start;
                    return;
                     */

            }
            telemetry.update();
        }
    }
}
