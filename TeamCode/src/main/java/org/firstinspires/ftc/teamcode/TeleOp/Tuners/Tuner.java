package org.firstinspires.ftc.teamcode.TeleOp.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class Tuner extends LinearOpMode {
    private MotorEx motor;
    private Motor.Encoder encoder;
    public static double p = 0.013;
    public static double i = 0.01;
    public static double d = 0.0007;
    public static double f = 0.0005;
    public static int targetPos = 1000;
    public double output;
    private PIDFController controller;

    @Override
    public void runOpMode(){
        motor = new MotorEx(hardwareMap, "LSL", Motor.GoBILDA.RPM_435);
        encoder = motor.encoder;
        motor.resetEncoder();
        motor.setRunMode(Motor.RunMode.RawPower);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDFController(p, i, d, f);
        telemetry.addData("Ready to Start!", "-");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            //motor.setTargetPosition((int) output);
            output = controller.calculate(motor.getCurrentPosition(), targetPos);
            motor.set(output);
            telemetry.addData("Target Pos: ", targetPos);
            telemetry.addData("Encoder Current Pos: ", encoder.getPosition());
            telemetry.addData("Motor Current Pos: ", motor.getCurrentPosition());
            telemetry.addData("Output: ", output);
            telemetry.addData("p: ", p);
            telemetry.addData("i: ", i);
            telemetry.addData("d: ", d);
            telemetry.update();

            controller.setPIDF(p, i, d, f);
        }
    }
}
