package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    }

    private Arm arm;
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
    private double gripPixelPos = 0.0;// TEMP: Check for the right value.
    private double releasePixelPos = 0.5;// TEMP: Check for the right value.
    private final double buffer = 5;
    private final double outakeSpeed = -0.4, intakeSpeed = 1.0;


    GamepadEx gamepad;
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public Cycle(HWMap hwMap, HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, FieldCentricDrive fieldCentricDrive) {
        this.gamepad = gamepad; // add control class to program
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.fieldCentricDrive = fieldCentricDrive;

        intakeMotor = hwMap.getIntakeMotor();
        outakeServoLeft = hwMap.getOutakeServoLeft();
        outakeServoRight = hwMap.getOutakeServoRight();
        colorSensorLeft = hwMap.getTrayLeftCS();
        colorSensorRight = hwMap.getTrayRightCS();
        arm = new Arm(telemetry, hardwareMap);
    }


    public void loop() {
        while (true) {
            gamepad.readButtons();

            switch (state) {
                case start:
                    //This is the starting state and when one of the buttons is pressed the FSM will move to its corresponding state.
                    //For example, if a is pressed it will move to the intake state.
                    telemetry.addData("in start in cycle", 1);
                    if (gamepad.isDown(GamepadKeys.Button.A)) {
                        telemetry.addData("a pressed in cycle", 1);
                        state = cycleFSM.intake;
                    }
                    if (gamepad.isDown(GamepadKeys.Button.B)) {
                        telemetry.addData("b pressed in cycle", 1);
                        //state = cycleFSM.ejection;
                    }
                    if (gamepad.isDown(GamepadKeys.Button.Y)) {
                        telemetry.addData("y pressed in cycle", 1);
                        //Will uncomment after slides have been tested
                        //state = cycleFSM.transfer;
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
                    state = cycleFSM.start;
                    break;
                case transfer:
                    arm.goToDeposit();
                    arm.updatePos();
                    if(arm.axonAtPos(arm.depositPos, buffer)){
                        state = cycleFSM.outake;
                    }else{
                        telemetry.addData("-", "Not at position to deposit");
                    }
                    intakeMotor.set(outakeSpeed);
                    if (gamepad.wasJustPressed(GamepadKeys.Button.B))
                        state = cycleFSM.start;
                    break;
                case outake:
                    telemetry.addData("-", "Ready to deposit");
                    if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        outakeServoLeft.setPosition(releasePixelPos);
                        outakeServoRight.setPosition(releasePixelPos);
                    }
                    intakeMotor.set(outakeSpeed);
                    if (gamepad.wasJustPressed(GamepadKeys.Button.B))
                        state = cycleFSM.start;
                    break;
            }
            telemetry.update();
            fieldCentricDrive.drive(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX(), HWMap.readFromIMU());

        }
    }

    private void Intake() {
        detectPixels();
        boolean axonAtPos = arm.axonAtPos(arm.depositPos, buffer);
        if (pixelInLeft)
            outakeServoLeft.setPosition(gripPixelPos);
        if (pixelInRight)
            outakeServoRight.setPosition(gripPixelPos);
        if (pixelInLeft && pixelInRight && linearSlidesAtPos && axonAtPos) {
            intakeMotor.set(outakeSpeed);
            //Enable state change after the slides have been added. 
            //DON'T THINK I WANT TO IMPLEMENT THE STATE CHANGE IN HERE, BUT LEFT IT JUST IN CASE.
            //state = cycleFSM.transfer;
        } else {
            intakeMotor.set(intakeSpeed);
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


