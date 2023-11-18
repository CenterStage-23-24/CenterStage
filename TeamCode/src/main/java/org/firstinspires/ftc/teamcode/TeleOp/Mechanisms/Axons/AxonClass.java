package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class AxonClass {
    private final CRServo axon;
    private final AnalogInput encoder;
    private double sign = 1;
    private double encoderOffset = 0.0;
    public AxonClass(CRServo axon, AnalogInput encoder, boolean inversePower, boolean inverseEncoder) {
        this.axon = axon;
        this.encoder = encoder;
        if (inversePower) {
            sign = -1;
        }
        if (inverseEncoder) {
            encoderOffset = 360;
        }
    }

    public void setPower(double power){
        axon.set(power * sign);
    }

    public double getPos() {
        return Math.abs(encoderOffset - (encoder.getVoltage() / 3.3 * 360));
    }



}