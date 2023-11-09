package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class Tuner extends LinearOpMode {
    private Motor motor;
    public static double p = 0.0;
    public static double i = 0.0;
    public static double d = 0.0;
    public static int targetPos = 0;
    private PIDController controller;

    @Override
    public void runOpMode(){
        motor = new Motor(hardwareMap, "LSL", Motor.GoBILDA.RPM_435);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDController(p, i, d);
        telemetry.addData("Ready to Start!", "-");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            double output = controller.calculate(motor.getCurrentPosition(), targetPos);
            motor.set(output);
            telemetry.addData("Target Pos: ", targetPos);
            telemetry.addData("Current Pos: ", motor.getCurrentPosition());
            telemetry.addData("output: ", output);
            telemetry.update();
        }
    }
}
