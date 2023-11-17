package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Cycle {

    public enum CycleFSM {
        start,
        extend,
        retract,
        outtakeLeft,
        outtakeRight
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
    private double leftGripPixelPos = 0;
    private double rightGripPixelPos = 1;
    private double leftReleasePixelPos = 0.5;
    private double rightReleasePixelPos = 0.5;
    private final double buffer = 20;
    private final double ejectSpeed = -0.4, intakeSpeed = 1.0;
    private boolean stopRequested = false;
    private boolean toTransfer = false;
    ElapsedTime bufferTime = new ElapsedTime();

    private double startTS;
    private double finalTS;

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
        arm.goToIntake();
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("-", Arm.targetPos);
        telemetry.update();
        bufferTime.reset();
    }


    public void loop() {
        while (true) {
            gamepad.readButtons();

            switch (state) {
                case start:
                    //This is the starting state and when one of the buttons is pressed the FSM will move to its corresponding state.
                    //For example, if a is pressed it will move to the intake state.
                    telemetry.addData("in start in cycle", 1);
                    if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                        telemetry.addData("Left-Bumper pressed in cycle", 1);
                        state = CycleFSM.outtakeLeft;
                    }
                    if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                        telemetry.addData("Left-Bumper in cycle", 1);
                        state = CycleFSM.outtakeRight;
                    }
                    if (gamepad.isDown(GamepadKeys.Button.Y)) {
                        telemetry.addData("y pressed in cycle", 1);
                        state = CycleFSM.extend;
                    }
                    if (gamepad.isDown(GamepadKeys.Button.A)) {
                        startTS = bufferTime.milliseconds();
                        telemetry.addData("b pressed in cycle", 1);
                        state = CycleFSM.retract;
                    }
                    break;
                case extend:
                    toTransfer = true;
                    slides.setTargetPos(slides.mmToTicks(50));
                    if (slides.atPos()) {
                        arm.goToDeposit();
                        state = CycleFSM.start;
                    }
                    telemetry.addData("atPos?", slides.atPos());
                    break;
                case outtakeLeft:
                    telemetry.addData("-", "Ready to deposit left");
                    outakeServoLeft.setPosition(leftReleasePixelPos);
                    toTransfer = true;
                    state = CycleFSM.start;
                    break;
                case outtakeRight:
                    telemetry.addData("-", "Ready to deposit right");
                    outakeServoRight.setPosition(rightReleasePixelPos);
                    toTransfer = true;
                    state = CycleFSM.start;
                    break;
                case retract:
                    toTransfer = true;
                    arm.goToIntake();
                    boolean armAtPos = arm.axonAtPos(Arm.intakePos, buffer);
                    if (armAtPos && waits()) {
                        slides.setTargetPos(0);
                        if (slides.atPos()) {
                            toTransfer = false;
                            state = CycleFSM.start;
                        }
                    }
                    break;
            }
            Intake();//Intake is out here because DRIVE TEAM wants the intake to run regardless
            telemetry.addData("OSL", outakeServoLeft.getPosition());
            telemetry.addData("OSR", outakeServoRight.getPosition());
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
            if (!toTransfer) {
                detectPixels();
                if (pixelInLeft)
                    outakeServoLeft.setPosition(leftGripPixelPos);
                if (pixelInRight)
                    outakeServoRight.setPosition(rightGripPixelPos);
                if ((!pixelInLeft || !pixelInRight)) {
                    intakeMotor.set(intakeSpeed);
                    telemetry.addData("power: ", intakeSpeed);
                } else {
                    intakeMotor.set(ejectSpeed);
                    telemetry.addData("power: ", ejectSpeed);
                }
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

    private boolean waits() {
        finalTS = bufferTime.milliseconds();
        if (finalTS - startTS == 750) {
            return true;
        }
        return false;

    }
}


