package org.firstinspires.ftc.teamcode.TeleOp.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Slides;
@Config
public class InheritedSlidesTuner extends Slides {
    private final Motor LSL;
    private final Motor LSR;
    public static double p = 0.013;
    public static double i = 0.01;
    public static double d = 0.0007;
    public static double f = 0.0005;

    /*
    TEST BENCH CONSTANTS:
    public static double p = 0.0032;
    public static double i = 0.01;
    public static double d = 0.0007;
    public static double f = 0;
     */

    public static int tolerance = 50;
    public static int targetPos;
    private final Telemetry telemetry;

    public InheritedSlidesTuner(HWMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
        this.LSL = super.LSL;
        this.LSR = super.LSR;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        targetPos = super.mmToTicks(50);
    }

    public void loop() {

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
