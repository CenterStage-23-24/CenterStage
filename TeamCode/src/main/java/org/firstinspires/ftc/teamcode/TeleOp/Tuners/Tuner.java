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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;

@Config
@TeleOp
public class Tuner extends LinearOpMode {
    private Motor LSL;
    private Motor LSR;
    public static double p = 0.013;
    public static double i = 0.01;
    public static double d = 0.0007;
    public static double f = 0.0005;
    public static int tolerance = 50;
    private PIDFController controller;
    public static int targetPos;
    private HWMap hwMap;
    private Telemetry telemetry;

    @Override
    public void runOpMode(){
        this.hwMap = new HWMap(telemetry, hardwareMap);
        LSL = hwMap.getLinearSlidesLeft();
        LSR = hwMap.getLinearSlidesRight();
        LSL.resetEncoder();
        LSR.resetEncoder();
        LSL.setRunMode(Motor.RunMode.RawPower);
        LSR.setRunMode(Motor.RunMode.RawPower);
        controller = new PIDFController(p, i, d, f);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        targetPos = mmToTicks(50);
        waitForStart();

        while (opModeIsActive()) {
            double output = controller.calculate(LSL.getCurrentPosition(), targetPos);
            LSL.set(output);
            LSR.set(output);

            telemetry.addData("target Pos", targetPos);
            telemetry.addData("LSL POS", LSL.getCurrentPosition());
            telemetry.addData("LSR POS", LSR.getCurrentPosition());
            telemetry.addData("tolerance", tolerance);
            telemetry.addData("p: ", p);
            telemetry.addData("i: ", i);
            telemetry.addData("d: ", d);
            telemetry.update();

            controller.setPIDF(p, i, d, f);
        }
    }

    public int mmToTicks(double cm) {
        double diameterOfSpool = 4.6;
        double ratio = 37.0 / 24.0;
        double ticks = (cm / (Math.PI * diameterOfSpool)) * ratio * LSL.getCPR();  // The 2 is due to the gear ratio.
        return (int) ticks;
    }
}
