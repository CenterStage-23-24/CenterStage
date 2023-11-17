package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slides {
    private Motor LSL;
    private Motor LSR;
    private Motor.Encoder LSL_encoder;
    private Motor.Encoder LSR_encoder;
    public static double p = 0.013;
    public static double i = 0.01;
    public static double d = 0.0007;
    public static double f = 0.0005;
    public static int tolerance = 50;
    private PIDFController controller;
    private int targetPos;
    private HWMap hwMap;
    private Telemetry telemetry;
    public Slides(HWMap hwMap, Telemetry telemetry){
        this.hwMap = hwMap;
        this.telemetry = telemetry;
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

    public void pid(){
        double output = controller.calculate(LSL.getCurrentPosition(), targetPos);
        LSL.set(output);
        LSR.set(output);

        telemetry.addData("target Pos", targetPos);
        telemetry.addData("LSL POS", LSL.getCurrentPosition());
        telemetry.addData("LSR POS", LSR.getCurrentPosition());
        telemetry.addData("tolerance", tolerance);
        /*
        Consolidated PID loop that conducts both slide and axon pid
        This allows us to concurrently move slides up and swing arm forward (or vice versa)
        Do not call until slides are above clearingHeight
        Currently commented out to test another alternative solution

        axonController.setPID(p, i, d);
        while(!(LSL.getCurrentPosition() <= targetPos + tolerance && LSL.getCurrentPosition() >= targetPos - tolerance)) {
            double output = controller.calculate(LSL.getCurrentPosition(), targetPos);
            LSL.set(output);
            LSR.set(output);

            double measuredPos = leftAxon.getPos();
            double delta = Math.min(normalizeRadiansTau(targetPos - measuredPos), 360 - normalizeRadiansTau(targetPos - measuredPos));
            int sign = (int) (-(Math.signum(normalizeRadiansTau(targetPos - measuredPos) - (360 - normalizeRadiansTau(targetPos - measuredPos)))));
            double error = delta * sign;

            double power = pidController.calculate(0, error);
            leftAxon.setPower(power);
            rightAxon.setPower(power);

            telemetry.addData("Power: ", power);
            telemetry.addData("Measured Pos: ", measuredPos);
            telemetry.addData("Target Pos: ", targetPos);
            telemetry.addData("Delta: ", delta);
            telemetry.addData("Sign: ", sign);
            telemetry.addData("Error: ", error);
            telemetry.update();
        }
         */
    }

    public void setTargetPos(int targetPos){
        this.targetPos = targetPos;
    }

    public boolean atPos(){

        return ((targetPos + tolerance) >= LSL.getCurrentPosition()) && ((targetPos - tolerance) <= LSL.getCurrentPosition());
    }
    public int mmToTicks(double mm){
        double diameterOfSpool = 46.0;
        double angleOfSlides = 60.0;
        double cmDiagonal = cm/sin(angleOfSlides);
        double rotations = cmDiagonal/(Math.PI * diameter * 0.5);// The 0.5 is due to the gear ratio. 
        double ticks = rotations * LSL.getCPR();
        return (int) ticks;
            
    }
}
