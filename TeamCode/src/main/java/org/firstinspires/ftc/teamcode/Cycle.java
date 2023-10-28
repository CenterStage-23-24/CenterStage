package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Cycle {

    public enum cycleFSM {
        start,
        intake,
        ejection,
        transfer,
        outtake
    }

    cycleFSM state = cycleFSM.start;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    private DcMotorEx intakeMotor;
    private DcMotorEx linearSlidesRight;
    private DcMotorEx linearSlidesLeft;
    private Servo outakeServoLeft;
    private Servo outakeServoRight;
    Gamepad gamepad;

    public Cycle (DcMotorEx intakeMotor, DcMotorEx linearSlidesRight, DcMotorEx linearSlidesLeft, Servo outakeServoLeft, Servo outakeServoRight, Gamepad gamepad) {
        HWMap hardware = new HWMap(telemetry, hardwareMap);
        intakeMotor = hardware.getIntakeMotor();
        linearSlidesRight = hardware.getLinearSlidesRight();
        linearSlidesLeft = hardware.getLinearSlidesLeft();
        outakeServoLeft = hardware.getOutakeServoLeft();
        outakeServoRight = hardware.getOutakeServoRight();
        gamepad = gamepad;
    }

    public void init() {


    }

    public void loop() {
        switch (state) {
            case start:
                state = cycleFSM.intake;
                break;
            case intake:
                if (gamepad.a) {
                    intakeMotor.setPower(0.5);
                }
                state = cycleFSM.ejection;
                break;
            case ejection:
                if (gamepad.options) {
                    intakeMotor.setPower(-0.5);
                }
                state = cycleFSM.transfer;
                break;
            case transfer:
                if (gamepad.y) {
                    linearSlidesLeft.setPower(0.5);
                    linearSlidesRight.setPower(0.5);
                }
                if (gamepad.b) {
                    linearSlidesLeft.setPower(-0.5);
                    linearSlidesRight.setPower(-0.5);
                }
                state = cycleFSM.outtake;
                break;
            case outtake:
                if (gamepad.left_bumper) {
                    outakeServoLeft.setPosition(0.75);
                }
                if (gamepad.right_bumper) {
                    outakeServoRight.setPosition(0.75);
                }
                state = cycleFSM.start;
                break;
            default:
                state = cycleFSM.start;
        }


    }
}