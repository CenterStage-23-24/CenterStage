package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp
public class Claws extends LinearOpMode {
    private HWMap hwMap;
    private Servo leftClaw;
    private Servo rightClaw;

    public static double openPos = 0.5;
    public static double closedPos = 0.0;

    public static int posVar = 0;

    @Override
    public void runOpMode() {
        try {
            hwMap = new HWMap(telemetry, hardwareMap);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            leftClaw = hwMap.getOutakeServoLeft();
            rightClaw = hwMap.getOutakeServoRight();

        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            double position;

            if (posVar == 0){
                position = openPos;
            }else{
                position = closedPos;
            }

            leftClaw.setPosition(position);
            rightClaw.setPosition(1 - position);

            telemetry.update();
        }
    }
}