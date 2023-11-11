package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleOp.HWMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Cycle {

    public enum cycleFSM {
        start,
        intake,
        ejection,
        transfer,
        outake,
        outakeReverse
    }

    cycleFSM state = cycleFSM.start;
    private Motor intakeMotor;
    private Servo outakeServoLeft;
    private Servo outakeServoRight;
    private RevColorSensorV3 colorSensorLeft;
    private RevColorSensorV3 colorSensorRight;
    private FieldCentricDrive fieldCentricDrive;
    boolean pixelInLeft = false;
    boolean pixelInRight = false;
    boolean linearSlidesAtPos = true; // Change to the linear slides threshold when Transfer is done
    boolean axonAtPos = true; //Change to axon threshold when Transfer is done.
    private double gripPixelPos = 0.8;// TEMP: Check for the right value.
    private double releasePixelPos = 0.0;// TEMP: Check for the right value.

    GamepadEx gamepad;
    Telemetry telemetry;

    public Cycle(HWMap hwMap, GamepadEx gamepad, Telemetry telemetry, FieldCentricDrive fieldCentricDrive) {
        this.gamepad = gamepad; // add control class to program
        this.telemetry = telemetry;
        this.fieldCentricDrive = fieldCentricDrive;

        intakeMotor = hwMap.getIntakeMotor();
        outakeServoLeft = hwMap.getOutakeServoLeft();
        outakeServoRight = hwMap.getOutakeServoRight();
        colorSensorLeft = hwMap.getTrayLeftCS();
        colorSensorRight = hwMap.getTrayRightCS();

    }


    public void loop() {
        while (true) {
            gamepad.readButtons();

            switch (state) {
                case start:
                    telemetry.addData("in start in cycle", 1);
                    if (gamepad.isDown(GamepadKeys.Button.A)) {
                        telemetry.addData("a pressed in cycle", 1);
                        state = cycleFSM.intake;
                    }
                    if (gamepad.isDown(GamepadKeys.Button.B)) {
                        telemetry.addData("b pressed in cycle", 1);
                        state = cycleFSM.ejection;
                    }
                    if (gamepad.isDown(GamepadKeys.Button.Y)) {
                        telemetry.addData("y pressed in cycle", 1);
                        state = cycleFSM.transfer;
                    }
                    if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                        telemetry.addData("left_bumper pressed in cycle", 1);
                        state = cycleFSM.outake;
                    }
                    if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                        state = cycleFSM.outakeReverse;
                    }

                    break;
                case intake:
                    if (gamepad.wasJustReleased(GamepadKeys.Button.A)) {
                        state = cycleFSM.start;
                    } else {
                        Intake();
                    }
                    break;
                case ejection:
                    //Ejection
                    break;
                case transfer:
                    //Transfer
                    break;
                case outake:
                    telemetry.addData("In outtake", 1);
                    outakeServoLeft.setPosition(0.75);
                    outakeServoRight.setPosition(0.75);
                    state = cycleFSM.start;
                    break;
                case outakeReverse:
                    outakeServoLeft.setPosition(0);
                    outakeServoRight.setPosition(0);
                    state = cycleFSM.start;
                    return;

            }
            telemetry.update();
            gamepad.readButtons();
            fieldCentricDrive.drive(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX(), HWMap.readFromIMU());

        }
    }

    private void Intake() {
        detectPixels();
        if (pixelInLeft)
            outakeServoLeft.setPosition(gripPixelPos);
        if (pixelInRight)
            outakeServoRight.setPosition(gripPixelPos);
        if (pixelInLeft && pixelInRight && linearSlidesAtPos && axonAtPos) {
            intakeMotor.set(-0.4);
        } else {
            intakeMotor.set(1);
        }
    }

    private void detectPixels() {
        double csLeftDistance = colorSensorLeft.getDistance(DistanceUnit.MM);
        double csRightDistance = colorSensorRight.getDistance(DistanceUnit.MM);
        if (csLeftDistance <= 30) {
            if (colorSensorLeft.green() > (colorSensorLeft.red() * 2)) {
                telemetry.addData("-", "Green pixel detected in the left compartment");
                pixelInLeft = true;
            } else if ((colorSensorLeft.blue() < colorSensorLeft.red()) && (colorSensorLeft.blue() < colorSensorLeft.green())) {
                telemetry.addData("-", "Yellow pixel detected in the left compartment");
                pixelInLeft = true;
            } else if ((colorSensorLeft.blue() > colorSensorLeft.red()) && (colorSensorLeft.blue() > colorSensorLeft.green())) {
                telemetry.addData("-", "Purple pixel detected in the left compartment");
                pixelInLeft = true;
            } else if (colorSensorLeft.green() >= 300) {
                telemetry.addData("-", "White pixel detected in the left compartment");
                pixelInLeft = true;
            }
        } else {
            telemetry.addData("-", "Nothing in the left compartment");
            pixelInLeft = false;
        }
        if (csRightDistance <= 28) {
            if (colorSensorRight.green() > (colorSensorRight.red() * 2)) {
                telemetry.addData("-", "Green pixel detected in the right compartment");
                pixelInRight = true;
            } else if ((colorSensorRight.blue() < colorSensorRight.red()) && (colorSensorRight.blue() < colorSensorRight.green())) {
                telemetry.addData("-", "Yellow pixel detected in the right compartment");
                pixelInRight = true;
            } else if ((colorSensorRight.blue() > colorSensorRight.red()) && (colorSensorRight.blue() > colorSensorRight.green())) {
                telemetry.addData("-", "Purple pixel detected in the right compartment");
                pixelInRight = true;
            } else if (colorSensorRight.green() > 300) {
                telemetry.addData("-", "White pixel detected in the right compartment");
                pixelInRight = true;
            }
        } else {
            telemetry.addData("-", "Nothing in right compartment");
            pixelInRight = false;
        }
    }

}


