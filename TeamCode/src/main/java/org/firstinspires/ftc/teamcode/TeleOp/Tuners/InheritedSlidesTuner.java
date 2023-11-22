package org.firstinspires.ftc.teamcode.TeleOp.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Slides;

public class InheritedSlidesTuner extends Slides {
    private Motor LSL;
    private Motor LSR;
    public static double p = 0.013;
    public static double i = 0.01;
    public static double d = 0.0007;
    public static double f = 0.0005;
    public static int tolerance = 50;
    public static int targetPos;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public InheritedSlidesTuner(HWMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
    }

    public void runOpMode() {
        HWMap hwMap = new HWMap(hardwareMap);
        LSL = hwMap.getLinearSlidesLeft();
        LSR = hwMap.getLinearSlidesRight();
        LSL.resetEncoder();
        LSR.resetEncoder();
        LSL.setRunMode(Motor.RunMode.RawPower);
        LSR.setRunMode(Motor.RunMode.RawPower);
        PIDFController controller = new PIDFController(p, i, d, f);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        targetPos = super.mmToTicks(50);

        while (true) {
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
}
