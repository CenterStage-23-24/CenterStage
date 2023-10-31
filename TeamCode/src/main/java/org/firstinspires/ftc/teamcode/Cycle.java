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
        telemetry.addLine("In cycle");
        telemetry.update();
        while (true) {
            switch (state) {
                case start:

                    if (gamepad.a) {
                        state = cycleFSM.intake;
                    }
                    if (gamepad.options) {
                        state = cycleFSM.ejection;
                    }
                    if (gamepad.y) {
                        state = cycleFSM.transfer;
                    }
                    if (gamepad.left_bumper) {
                        state = cycleFSM.outtake;
                    }
                    break;
                case intake:
                        intakeMotor.setPower(0.5);
                        state = cycleFSM.start;
                   break;
                case ejection:
                        intakeMotor.setPower(-0.5);
                    state = cycleFSM.start;
                    break;
                case transfer:
                        linearSlidesLeft.setPower(0.5);
                        linearSlidesRight.setPower(0.5);
                    state = cycleFSM.start;
                    break;
                case outtake:
                        outakeServoLeft.setPosition(0.75);
                        outakeServoRight.setPosition(0.75);
                    state = cycleFSM.start;
                    return;
            }
        }


    }
}