package org.firstinspires.ftc.teamcode.Auto.RoadRunner;

public class MotorPowerVector {
    
    // Variables are listed in RoadRunner order
    // (see definition of SampleMecanumDrive.setMotorPowers)
    private double leftFrontPower;
    private double leftBackPower;
    private double rightBackPower;
    private double rightFrontPower;
    
    public MotorPowerVector(double v, double v1, double v2, double v3) {
        leftFrontPower = v;
        leftBackPower = v1;
        rightBackPower = v2;
        rightFrontPower = v3;
    }
    
    public MotorPowerVector scale(double factor) {
        return new MotorPowerVector(
                leftFrontPower * factor, 
                leftBackPower * factor,
                rightBackPower * factor,
                rightFrontPower * factor
        );
    }

    public double getLeftFrontPower() {
        return leftFrontPower;
    }

    public double getLeftBackPower() {
        return leftBackPower;
    }

    public double getRightBackPower() {
        return rightBackPower;
    }

    public double getRightFrontPower() {
        return rightFrontPower;
    }
}
