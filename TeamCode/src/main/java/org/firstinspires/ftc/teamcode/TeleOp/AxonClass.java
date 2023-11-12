package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AxonClass {
    private CRServo axon;
    private AnalogInput encoder;
    private boolean inverse;
    private PIDController pidController;
    private double measuredPos;
    private double error = 0 , lastError = 0;
    private double integralSum = 0;
    private ElapsedTime timer = new ElapsedTime();
    public AxonClass(CRServo axon, AnalogInput encoder, boolean inverse, PIDController pidController) {
        this.axon = axon;
        this.encoder = encoder;
        this.inverse = inverse;
        this.pidController = pidController;
    }

    public double setPos(double targetPos) {
        double power = pidController.calculate(0, targetPos);
        axon.set(power);
        return power;
    }

    public void setPID(double p, double i, double d) {
        pidController.setPID(p, i, d);
    }

    public double[] getPID() {
        return new double[]{pidController.getP(), pidController.getI(), pidController.getD()};
    }

    public double getPos() {
        return encoder.getVoltage() / 3.3 * 360;
    }



}