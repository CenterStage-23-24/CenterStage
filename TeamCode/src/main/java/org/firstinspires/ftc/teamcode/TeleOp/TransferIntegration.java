package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TransferIntegration extends LinearOpMode {
    private HWMap hwMap;
    private GamepadEx gamePad1;
    private Servo leftClaw;
    private Servo rightClaw;
    private Arm arm;

    private double p = 0.0056, i = 0.003, d = 0.015;
    private  double openPos = 0.5;
    private  double closedPos = 0.0;
    private  double pos = 0.0;


    @Override
    public void runOpMode() {
        try {
            hwMap = new HWMap(telemetry, hardwareMap);
            gamePad1 = new GamepadEx(gamepad1);

            arm = new Arm(telemetry, hardwareMap);

            leftClaw = hwMap.getOutakeServoLeft();
            rightClaw = hwMap.getOutakeServoRight();
        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            gamePad1.readButtons();
            if (gamePad1.wasJustPressed(GamepadKeys.Button.Y)) {
                arm.goToDeposit();
            } else if (gamePad1.wasJustPressed(GamepadKeys.Button.A)) {
                arm.goToIntake();
            }
            if (gamePad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                pos = openPos;
                leftClaw.setPosition(pos);
                rightClaw.setPosition(1 - pos);
            }else if(gamePad1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                pos = closedPos;
                leftClaw.setPosition(pos);
                rightClaw.setPosition(1 - pos);
            }
            arm.updatePos();
            telemetry.update();
        }
    }
}
