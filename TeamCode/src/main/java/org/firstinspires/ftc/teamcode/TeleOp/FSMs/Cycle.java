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
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Slides;

public class Cycle {

    public enum CycleFSM {
        start,
        extend,
        retract,
        outtakeLeft,
        outtakeRight
    }


    CycleFSM state = CycleFSM.start;
    private final Motor intakeMotor;
    private final Servo outakeServoLeft;
    private final Servo outakeServoRight;
    private final RevColorSensorV3 colorSensorLeft;
    private final RevColorSensorV3 colorSensorRight;
    boolean pixelInLeft = false;
    boolean pixelInRight = false;
    private final double leftReleasePixelPos = 0.5;
    private final double rightReleasePixelPos = 0.5;
    private boolean toTransfer = false;
    private final ElapsedTime bufferTime = new ElapsedTime();


    private double startTS;

    private final GamepadEx gamepad;
    private Telemetry telemetry;

    private Slides slides;
    private Arm arm;

    public Cycle(HWMap hwMap, GamepadEx gamepad, Telemetry telemetry, Slides slides, Arm arm) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.slides = slides;
        this.arm = arm;

        intakeMotor = hwMap.getIntakeMotor();
        outakeServoLeft = hwMap.getOuttakeServoLeft();
        outakeServoRight = hwMap.getOuttakeServoRight();
        colorSensorLeft = hwMap.getTrayLeftCS();
        colorSensorRight = hwMap.getTrayRightCS();

        slides = new Slides(hwMap, telemetry);
        arm = new Arm(hwMap, telemetry);

        //Forces arm to init to intake pos when targetPos is initialized to outtakePos
        arm.goToIntake();

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bufferTime.reset();

        outakeServoLeft.setPosition(leftReleasePixelPos);
        outakeServoRight.setPosition(rightReleasePixelPos);

        telemetry.addData("INIT: ", "Cycle");
        telemetry.update();
    }


    public void loop() {
        telemetry.addData("atpos test: ", slides.atPos());
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

    }



    private boolean delay() {
        double finalTS = bufferTime.milliseconds();
        int ms = 750;//600
        return (finalTS - startTS) >= ms;
    }

    public boolean getToTransfer(){
        return toTransfer;
    }
}


