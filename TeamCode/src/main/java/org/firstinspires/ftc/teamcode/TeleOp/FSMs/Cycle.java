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
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.TransferController;

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
    private CycleFSM state = CycleFSM.start;
    private TransferController transferController;

    private final Servo outakeServoLeft;
    private final Servo outakeServoRight;

    private final double leftReleasePixelPos = 0.5;
    private final double rightReleasePixelPos = 0.5;
    private boolean toTransfer = false;


    private final GamepadEx gamepad;
    private Telemetry telemetry;

    public Cycle(HWMap hwMap, GamepadEx gamepad, Telemetry telemetry, FieldCentricDrive fieldCentricDrive) {
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        outakeServoLeft = hwMap.getOuttakeServoLeft();
        outakeServoRight = hwMap.getOuttakeServoRight();

        slides = new Slides(hwMap, telemetry);
        arm = new Arm(hwMap, telemetry);
        transferController = new TransferController(arm, slides, telemetry);

        //Forces arm to init to intake pos when targetPos is initialized to outtakePos
        arm.goToIntake();

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
                        telemetry.addData("b pressed in cycle", 1);
                        state = CycleFSM.retract;
                    }
                    break;

                case extend:
                  toTransfer = true;
                    if(transferController.extend()){
                        state = CycleFSM.start;
                    }
                    break;

                case retract:
                    toTransfer = true;
                    if(transferController.retract()){
                        state = CycleFSM.start;
                        toTransfer = false;
                    }
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
            }

    }

    public boolean getToTransfer(){
        return toTransfer;
    }
}


