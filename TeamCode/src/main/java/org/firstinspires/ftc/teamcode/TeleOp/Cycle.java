package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Cycle {

    public enum CycleFSM {
        start,
        intake,
        ejection,
        transfer,
        outtake,
    }

    private Arm arm;
    private Slides slides;
    CycleFSM state = CycleFSM.start;
    private Motor intakeMotor;
    private Servo outakeServoLeft;
    private Servo outakeServoRight;
    private RevColorSensorV3 colorSensorLeft;
    private RevColorSensorV3 colorSensorRight;
    private FieldCentricDrive fieldCentricDrive;
    boolean pixelInLeft = false;
    boolean pixelInRight = false;
    private double gripPixelPos = 0.0;// TEMP: Check for the right value.
    private double releasePixelPos = 0.5;// TEMP: Check for the right value.
    private final double buffer = 20;
    private final double ejectSpeed = -0.4, intakeSpeed = 1.0;
    private boolean stopRequested = false;
    private boolean toTransfer = false;


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
        slides = new Slides(hwMap, telemetry);
        arm = new Arm(hwMap, telemetry);
    }


    public void loop() {
        while (true) {
            gamepad.readButtons();

            switch (state) {
                case start:
                    //This is the starting state and when one of the buttons is pressed the FSM will move to its corresponding state.
                    //For example, if a is pressed it will move to the intake state.
                    telemetry.addData("in start in cycle", 1);
                    if (gamepad.isDown(GamepadKeys.Button.B)) {
                        telemetry.addData("b pressed in cycle", 1);
                        state = CycleFSM.outtake;
                    }
                    if (gamepad.isDown(GamepadKeys.Button.Y)) {
                        telemetry.addData("y pressed in cycle", 1);
                        //Will uncomment after slides have been tested
                        state = CycleFSM.transfer;
                    }
                    if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                        state = CycleFSM.outtake;
                    }
                    break;
                case ejection:
                    intakeMotor.set(ejectSpeed);
                    state = CycleFSM.start;
                    break;
                case transfer:
                    toTransfer = true;
                    slides.setTargetPos(1000);
                    if (slides.atPos()) {
                        arm.goToDeposit();
                        state = CycleFSM.start;
                    }
                    telemetry.addData("atPos?", slides.atPos());
                    break;
                case outtake:
                    telemetry.addData("-", "Ready to deposit");
                    if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        outakeServoLeft.setPosition(releasePixelPos);
                        outakeServoRight.setPosition(1 - releasePixelPos);
                        toTransfer = false;
                    }
                    if (gamepad.wasJustPressed(GamepadKeys.Button.B))
                        state = CycleFSM.start;
                    break;
            }
            Intake();//Intake is out here because DRIVE TEAM wants the intake to run regardless
            telemetry.update();
            fieldCentricDrive.drive(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX(), HWMap.readFromIMU());
            slides.pid();
            arm.updatePos();

        }
    }

    private void Intake() {
        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            stopRequested = !stopRequested;

        if (!stopRequested) {
            detectPixels();
            boolean axonAtPos = arm.axonAtPos(Arm.intakePos, buffer);
            boolean linearSlidesAtPos = slides.atPos(); // Change to the linear slides threshold when Transfer is done
            if (pixelInLeft)
                outakeServoLeft.setPosition(gripPixelPos);
            if (pixelInRight)
                outakeServoRight.setPosition(gripPixelPos);
            if ((!pixelInLeft || !pixelInRight) && !toTransfer) {
                intakeMotor.set(intakeSpeed);
                telemetry.addData("power: ", intakeSpeed);
            } else {
                intakeMotor.set(ejectSpeed);
                telemetry.addData("power: ", ejectSpeed);
            }
        } else {
            intakeMotor.set(0);
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


