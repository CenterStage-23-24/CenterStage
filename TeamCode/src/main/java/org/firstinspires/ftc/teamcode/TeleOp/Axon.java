package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.FtcDashboard;

import org.checkerframework.checker.units.qual.A;

import java.net.CacheRequest;

@Config
@TeleOp
public class Axon extends LinearOpMode {
    private HWMap hwMap;
    private CRServo leftServo;
    private AnalogInput leftEncoder;
    private AxonClass leftAxon;
    private PIDController pidController;
    public static double p = 0.008, i = 0.0, d = 0.00035;
    public static double targetPos = 50;

    @Override
    public void runOpMode() {
        try {
            hwMap = new HWMap(telemetry, hardwareMap);

            leftServo = hwMap.getAxonServoLeft();
            leftEncoder = hwMap.getAxonAnalogLeft();

            pidController = new PIDController(p, i, d);
            leftAxon = new AxonClass(leftServo, leftEncoder, false, true);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            pidController.setPID(p, i, d);
            double measuredPos = leftAxon.getPos();

            double delta = Math.min(normalizeRadiansTau(targetPos - measuredPos), 360 - normalizeRadiansTau(targetPos - measuredPos));
            int sign = (int) (-(Math.signum(normalizeRadiansTau(targetPos - measuredPos) - (360 - normalizeRadiansTau(targetPos - measuredPos)))));
            double error = delta * sign;

            double power = pidController.calculate(0, error);
            leftAxon.setPower(power);

            telemetry.addData("Power: ", power);
            telemetry.addData("Measured Pos: ", measuredPos);
            telemetry.addData("Target Pos: ", targetPos);
            telemetry.addData("Delta: ", delta);
            telemetry.addData("Sign: ", sign);
            telemetry.addData("Error: ", error);
            telemetry.update();
        }
    }

    public double normalizeRadiansTau(double angle) {
        return (angle + 360) % 360;
    }
}