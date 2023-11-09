package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
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

    private Motor intakeMotor;
    private Motor linearSlidesRight;
    private Motor linearSlidesLeft;
    private Servo outakeServoLeft;
    private Servo outakeServoRight;
    private FieldCentricFTCLib fieldCentricDrive;
    GamepadEx gamepad;
    Telemetry telemetry;

    public Cycle (HWMap hardware, GamepadEx gamepad, Telemetry telemetry, HardwareMap hardwareMap) {
        intakeMotor = hardware.getIntakeMotor();
        linearSlidesRight = hardware.getLinearSlidesRight();
        linearSlidesLeft = hardware.getLinearSlidesLeft();
        outakeServoLeft = hardware.getOutakeServoLeft();
        outakeServoRight = hardware.getOutakeServoRight();
         fieldCentricDrive = new FieldCentricFTCLib(telemetry,hardwareMap);
        this.gamepad = gamepad; // add control class to program
        this.telemetry = telemetry;
        //telemetry.addData("gamepad1: ", gamepad.toString());
        telemetry.update();
    }


    public void loop() throws InterruptedException {
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
                    if (gamepad.getButton(GamepadKeys.Button.B)) {
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
                    intakeMotor.set(0.5);
                    Thread.sleep(5);
                    intakeMotor.set(0);
                    state = cycleFSM.start;
                    break;

                case ejection:
                    telemetry.addData("In ejection", 1);
                    intakeMotor.set(-0.5);
                    Thread.sleep(5);
                    intakeMotor.set(0);
                    state = cycleFSM.start;
                    break;
                case transfer:
                    telemetry.addData("In transfer", 1);
                    linearSlidesLeft.set(0.5);
                    linearSlidesRight.set(0.5);
                    Thread.sleep(5);
                    linearSlidesLeft.set(0);
                    linearSlidesRight.set(0);
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
            //drivetrain code here
            gamepad.readButtons();


            fieldCentricDrive.drive(gamepad);

        }
    }
}
