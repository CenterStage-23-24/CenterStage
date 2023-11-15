package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class Slides {
    private Motor LSL;
    private Motor LSR;
    private Motor.Encoder LSL_encoder;
    private Motor.Encoder LSR_encoder;
    public static double p = 0.013;
    public static double i = 0.01;
    public static double d = 0.0007;
    public static double f = 0.0005;
    public static int tolerance = 10;
    private PIDFController controller;
    private HWMap hwMap;
    public Slides(HWMap hwMap){
        this.hwMap = hwMap;
        LSL = hwMap.getLinearSlidesLeft();
        LSR = hwMap.getLinearSlidesRight();
        LSL_encoder = LSL.encoder;
        LSR_encoder = LSR.encoder;
        LSL.resetEncoder();
        LSR.resetEncoder();
        LSL.setRunMode(Motor.RunMode.RawPower);
        LSR.setRunMode(Motor.RunMode.RawPower);
        controller = new PIDFController(p, i, d, f);
    }

    public void pid(int targetPos){
        while(!(LSL.getCurrentPosition() <= targetPos + tolerance && LSL.getCurrentPosition() >= targetPos - tolerance)) {
            double output = controller.calculate(LSL.getCurrentPosition(), targetPos);
            LSL.set(output);
            LSR.set(output);
        }
    }
}
