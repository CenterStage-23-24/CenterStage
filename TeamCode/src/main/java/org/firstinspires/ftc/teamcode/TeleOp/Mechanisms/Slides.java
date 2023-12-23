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
    private double output = 0;

/*
//TEST BENCH CONSTANTS:
    public static double P = 0.0027;
    public static double I = 0.01;
    public static double D = 0.0004;
    public static double F = 0;
*/
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

    public void pid(boolean toTransfer) {

        if(toTransfer)
            output = controller.calculate(LSL.getCurrentPosition(), targetPos);
        else
            output = controller.calculate(LSL.getCurrentPosition(), 0);

        LSL.set(output);
        LSR.set(output);

        telemetry.addData("target Pos", targetPos);
        telemetry.addData("LSL POS", LSL.getCurrentPosition());
        telemetry.addData("LSR POS", LSR.getCurrentPosition());
        telemetry.addData("tolerance", tolerance);
        telemetry.addData("LSL cm", ticksToCm(LSL.getCurrentPosition()));
        telemetry.addData("LSR cm", ticksToCm(LSR.getCurrentPosition()));
        telemetry.addData("OUTPUT: ", output);
    }


    public void setTargetPos(int targetPos) {
        this.targetPos = targetPos;
    }

    public boolean atPos() {
        return ((targetPos + tolerance) >= LSL.getCurrentPosition()) && ((targetPos - tolerance) <= LSL.getCurrentPosition());
    }
    public boolean atSpecificPos(double specificPos) {
        return ((specificPos + tolerance) >= LSL.getCurrentPosition()) && ((specificPos - tolerance) <= LSL.getCurrentPosition());
    }

    public int mmToTicks(double cm) {
        double diameterOfSpool = 4.6;
        double ratio = 37.0 / 24.0;
        double ticks = (cm / (Math.PI * diameterOfSpool)) * ratio * LSL.getCPR();
        return (int) ticks;

    }

    public double ticksToCm(int ticks){
        double diameterOfSpool = 4.6;
        double ratio = 37.0 / 24.0;
        return (ticks / (LSL.getCPR() * ratio)) * Math.PI * diameterOfSpool;
    }
}
