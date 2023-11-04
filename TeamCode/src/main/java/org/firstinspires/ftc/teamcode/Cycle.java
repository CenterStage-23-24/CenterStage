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
        outtake,
        outtakeReverse
    }

    cycleFSM state = cycleFSM.start;
    private DcMotorEx intakeMotor;
    private DcMotorEx linearSlidesRight;
    private DcMotorEx linearSlidesLeft;
    private Servo outakeServoLeft;
    private Servo outakeServoRight;
    Controller gamepad1;
    Telemetry telemetry;

    public Cycle (HWMap hardware, Controller gamepad1, Telemetry telemetry) {
        intakeMotor = hardware.getIntakeMotor();
        linearSlidesRight = hardware.getLinearSlidesRight();
        linearSlidesLeft = hardware.getLinearSlidesLeft();
        outakeServoLeft = hardware.getOutakeServoLeft();
        outakeServoRight = hardware.getOutakeServoRight();
        this.gamepad1 = gamepad1; // add control class to program
        this.telemetry = telemetry;
        //telemetry.addData("gamepad1: ", gamepad.toString());
        telemetry.update();
    }


    public void loop() throws InterruptedException {
        while(true) {
            gamepad1.gamepadEx1.readButtons();
            telemetry.addData("In cycle", 1);
            telemetry.addData("in while loop in cycle", 1);
            switch (state) {
                case start:
                    telemetry.addData("in start in cycle", 1);
                    if (gamepad1.a) {
                        telemetry.addData("a pressed in cycle", 1);
                        state = cycleFSM.intake;
                    }

                    if (gamepad1.b) {
                        telemetry.addData("b pressed in cycle", 1);
                        state = cycleFSM.ejection;
                    }
                    if (gamepad1.y) {
                        telemetry.addData("y pressed in cycle", 1);
                        state = cycleFSM.transfer;
                    }
                    if (gamepad1.LBumper) {
                        telemetry.addData("left_bumper pressed in cycle", 1);
                        state = cycleFSM.outtake;
                    }
                    if (gamepad1.RBumper) {
                        state = cycleFSM.outtakeReverse;
                    }

                    break;
                case intake:
                    telemetry.addData("In intake", 1);
                    intakeMotor.setPower(0.5);
                    Thread.sleep(5);
                    intakeMotor.setPower(0);
                    state = cycleFSM.start;
                    break;

                case ejection:
                    telemetry.addData("In ejection", 1);
                    intakeMotor.setPower(-0.5);
                    Thread.sleep(5);
                    intakeMotor.setPower(0);
                    state = cycleFSM.start;
                    break;
                case transfer:
                    telemetry.addData("In transfer", 1);
                    linearSlidesLeft.setPower(0.5);
                    linearSlidesRight.setPower(0.5);
                    Thread.sleep(5);
                    linearSlidesLeft.setPower(0);
                    linearSlidesRight.setPower(0);
                    state = cycleFSM.start;
                    break;
                case outtake:
                    telemetry.addData("In outtake", 1);
                    outakeServoLeft.setPosition(0.75);
                    outakeServoRight.setPosition(0.75);
                    state = cycleFSM.start;
                    break;
                case outtakeReverse:
                    outakeServoLeft.setPosition(0);
                    outakeServoRight.setPosition(0);
                    state = cycleFSM.start;
                    return;

            }
            telemetry.update();
        }
    }
}
