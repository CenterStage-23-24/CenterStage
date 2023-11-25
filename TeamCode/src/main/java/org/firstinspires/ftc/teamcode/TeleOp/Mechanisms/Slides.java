package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slides {
    protected final Motor LSL;
    protected final Motor LSR;
    private static final double P = 0.013;
    private static final double I = 0.01;
    private static final double D = 0.0007;
    private static final double F = 0.0005;
    private final int tolerance = 50;
    protected final PIDFController controller;
    private int targetPos;
    private final Telemetry telemetry;

    public Slides(HWMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        LSL = hwMap.getLinearSlidesLeft();
        LSR = hwMap.getLinearSlidesRight();
        LSL.resetEncoder();
        LSR.resetEncoder();
        LSL.setRunMode(Motor.RunMode.RawPower);
        LSR.setRunMode(Motor.RunMode.RawPower);
        controller = new PIDFController(P, I, D, F);
    }

    public void pid() {
        double output = controller.calculate(LSL.getCurrentPosition(), targetPos);
        LSL.set(output);
        LSR.set(output);

        telemetry.addData("target Pos", targetPos);
        telemetry.addData("LSL POS", LSL.getCurrentPosition());
        telemetry.addData("LSR POS", LSR.getCurrentPosition());
        telemetry.addData("tolerance", tolerance);
    }

    public void setTargetPos(int targetPos) {
        this.targetPos = targetPos;
    }

    public boolean atPos() {
        return ((targetPos + tolerance) >= LSL.getCurrentPosition()) && ((targetPos - tolerance) <= LSL.getCurrentPosition());
    }

    protected int mmToTicks(int cm) {
        double diameterOfSpool = 4.6;
        double ratio = 37.0 / 24.0;
        double ticks = (cm / (Math.PI * diameterOfSpool)) * ratio * LSL.getCPR();
        return (int) ticks;

    }
}
