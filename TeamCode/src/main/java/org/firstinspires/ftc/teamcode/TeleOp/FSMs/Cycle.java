package org.firstinspires.ftc.teamcode.TeleOp.FSMs;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Slides;

public class Cycle {

    public enum CycleFSM {
        start,
        extend,
        retract,
        outtakeLeft,
        outtakeRight
    }

    private final Arm arm;
    private final Slides slides;
    CycleFSM state = CycleFSM.start;
    private final Motor intakeMotor;
    private final RevColorSensorV3 colorSensorLeft;
    private final RevColorSensorV3 colorSensorRight;
    private final FieldCentricDrive fieldCentricDrive;
    private final Servo outtakeServoLeft;
    private final Servo outtakeServoRight;
    boolean pixelInLeft = false;
    boolean pixelInRight = false;
    private boolean stopRequested = false;
    private boolean toTransfer = false;
    private final ElapsedTime bufferTime = new ElapsedTime();

    private double startTS;

    private final GamepadEx gamepad;
    private Telemetry telemetry;
    private Outtake outtake;
    public Cycle(HWMap hwMap, GamepadEx gamepad, Telemetry telemetry, FieldCentricDrive fieldCentricDrive) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.fieldCentricDrive = fieldCentricDrive;

        intakeMotor = hwMap.getIntakeMotor();
        colorSensorLeft = hwMap.getTrayLeftCS();
        colorSensorRight = hwMap.getTrayRightCS();
        outtakeServoLeft = hwMap.getOuttakeServoLeft();
        outtakeServoRight = hwMap.getOuttakeServoRight();

        outtake = new Outtake(hwMap);
        slides = new Slides(hwMap, telemetry);
        arm = new Arm(hwMap, telemetry);

        //Forces arm to init to intake pos when targetPos is initialized to outtakePos
        arm.goToIntake();

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bufferTime.reset();

        telemetry.addData("INIT: ", "Cycle");
        telemetry.update();
    }


    public void loop() {
        while (true) {
            gamepad.readButtons();

            double buffer = 20;
            int height = 50;

            switch (state) {
                case start:
                    //Outtake Left
                    telemetry.addData("in start in cycle", 1);
                    if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                        telemetry.addData("Left-Bumper pressed in cycle", 1);
                        state = CycleFSM.outtakeLeft;
                    }

                    //Outtake Right
                    if (gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                        telemetry.addData("Left-Bumper in cycle", 1);
                        state = CycleFSM.outtakeRight;
                    }

                    //Extend
                    if (gamepad.isDown(GamepadKeys.Button.Y)) {
                        telemetry.addData("y pressed in cycle", 1);
                        state = CycleFSM.extend;
                    }

                    //Retract
                    if (gamepad.isDown(GamepadKeys.Button.A)) {
                        startTS = bufferTime.milliseconds();
                        telemetry.addData("b pressed in cycle", 1);
                        state = CycleFSM.retract;
                    }
                    break;

                case extend:
                    toTransfer = true;
                    slides.setTargetPos(slides.mmToTicks(height));
                    if (slides.atPos()) {
                        arm.goToDeposit();
                        state = CycleFSM.start;
                    }
                    telemetry.addData("atPos?", slides.atPos());
                    break;

                case outtakeLeft:
                    outtake.releaseLeft();
                    toTransfer = true;
                    state = CycleFSM.start;
                    break;

                case outtakeRight:
                    outtake.releaseRight();
                    toTransfer = true;
                    state = CycleFSM.start;
                    break;

                case retract:
                    toTransfer = true;
                    arm.goToIntake();
                    boolean armAtPos = arm.axonAtPos(Arm.intakePos, buffer);
                    if (armAtPos && delay()) {
                        height = 0;
                        slides.setTargetPos(height);
                        if (slides.atPos()) {
                            toTransfer = false;
                            state = CycleFSM.start;
                        }
                    }
                    break;
            }

            //Intake is out here because DRIVE TEAM wants the intake to run regardless
            intake();

            fieldCentricDrive.drive(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX(), HWMap.readFromIMU());
            slides.pid();
            arm.updatePos();
            telemetry.update();
        }
    }

    private void intake() {
        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            stopRequested = !stopRequested;

        if (!stopRequested) {
            double ejectSpeed = -0.4;
            double intakeSpeed = 1.0;


            if (!toTransfer) {
                detectPixels();
                if (pixelInLeft)
                    outtake.gripLeft();
                if (pixelInRight)
                    outtake.gripRight();
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
        final int rightDistance = 28;
        final int leftDistance = 30;
        final int greenThreshold = 300;
        if (csLeftDistance <= leftDistance) {
            if (colorSensorLeft.green() > (colorSensorLeft.red() * 2)) {
                telemetry.addData("-", "Green pixel detected in the left compartment");
                pixelInLeft = true;
            } else if ((colorSensorLeft.blue() < colorSensorLeft.red()) && (colorSensorLeft.blue() < colorSensorLeft.green())) {
                telemetry.addData("-", "Yellow pixel detected in the left compartment");
                pixelInLeft = true;
            } else if ((colorSensorLeft.blue() > colorSensorLeft.red()) && (colorSensorLeft.blue() > colorSensorLeft.green())) {
                telemetry.addData("-", "Purple pixel detected in the left compartment");
                pixelInLeft = true;
            } else if (colorSensorLeft.green() >= greenThreshold) {
                telemetry.addData("-", "White pixel detected in the left compartment");
                pixelInLeft = true;
            }
        } else {
            telemetry.addData("-", "Nothing in the left compartment");
            pixelInLeft = false;
        }
        if (csRightDistance <= rightDistance) {
            if (colorSensorRight.green() > (colorSensorRight.red() * 2)) {
                telemetry.addData("-", "Green pixel detected in the right compartment");
                pixelInRight = true;
            } else if ((colorSensorRight.blue() < colorSensorRight.red()) && (colorSensorRight.blue() < colorSensorRight.green())) {
                telemetry.addData("-", "Yellow pixel detected in the right compartment");
                pixelInRight = true;
            } else if ((colorSensorRight.blue() > colorSensorRight.red()) && (colorSensorRight.blue() > colorSensorRight.green())) {
                telemetry.addData("-", "Purple pixel detected in the right compartment");
                pixelInRight = true;
            } else if (colorSensorRight.green() >= greenThreshold) {
                telemetry.addData("-", "White pixel detected in the right compartment");
                pixelInRight = true;
            }
        } else {
            telemetry.addData("-", "Nothing in right compartment");
            pixelInRight = false;
        }
    }

    private boolean delay() {
        double finalTS = bufferTime.milliseconds();
        int ms = 750;//600
        return (finalTS - startTS) >= ms;
    }
}


