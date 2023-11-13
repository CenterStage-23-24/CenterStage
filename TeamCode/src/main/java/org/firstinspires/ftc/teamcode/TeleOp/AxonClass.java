package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AxonClass {
    private CRServo axon;
    private AnalogInput encoder;
    private double measuredPos;
    private double sign = 1;
    public AxonClass(CRServo axon, AnalogInput encoder, boolean inverse) {
        this.axon = axon;
        this.encoder = encoder;
        if (inverse) {
            sign = -1;
        }
    }

    public void setPower(double power){
        axon.set(power * sign);
    }

    public double getPos() {
        return encoder.getVoltage() / 3.3 * 360;
    }



}