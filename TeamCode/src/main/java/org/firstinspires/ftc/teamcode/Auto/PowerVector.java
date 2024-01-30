package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

public class PowerVector {
    
    // Variables are listed in RoadRunner order
    // (see definition of SampleMecanumDrive.setMotorPowers)
    private double leftFrontPower;
    private double leftBackPower;
    private double rightBackPower;
    private double rightFrontPower;
    
    public PowerVector(double v, double v1, double v2, double v3) {
        leftFrontPower = v;
        leftBackPower = v1;
        rightBackPower = v2;
        rightFrontPower = v3;
    }

    public double maxMagnitudeValue() {
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        return Math.max(max, Math.abs(rightFrontPower));
    }

    public PowerVector add(PowerVector other) {
        return new PowerVector(
                leftFrontPower + other.getLeftFrontPower(),
                leftBackPower + other.getLeftBackPower(),
                rightBackPower + other.getRightBackPower(),
                rightFrontPower + other.getRightFrontPower()
        );
    }
    
    public PowerVector scale(double factor) {
        return new PowerVector(
                leftFrontPower * factor, 
                leftBackPower * factor,
                rightBackPower * factor,
                rightFrontPower * factor
        );
    }

    public PowerVector clip(PowerVector threshold) {
        double tLeftFrontPower = threshold.getLeftFrontPower();
        double tLeftBackPower = threshold.getLeftBackPower();
        double tRightBackPower = threshold.getRightBackPower();
        double tRightFrontPower = threshold.getRightFrontPower();

        return new PowerVector(
                Math.min(leftFrontPower, tLeftFrontPower),
                Math.min(leftBackPower, tLeftBackPower),
                Math.min(rightBackPower, tRightBackPower),
                Math.min(rightFrontPower, tRightFrontPower)
        );
    }

    public PowerVector normalize(double maxMagnitude) {
        double adjustmentFactor = maxMagnitude / maxMagnitudeValue();

        return new PowerVector(
                leftFrontPower * adjustmentFactor,
                leftBackPower * adjustmentFactor,
                rightBackPower * adjustmentFactor,
                rightFrontPower * adjustmentFactor
        );
    }

    public PowerVector limit(double maxMagnitude) {
        if (maxMagnitude > maxMagnitudeValue())
            return normalize(maxMagnitude);
        else
            return this;
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

    @NonNull
    public String toString() {
        return "leftFrontPower: " + leftFrontPower
                + "\nleftBackPower: " + leftBackPower
                + "\nrightBackPower: " + rightBackPower
                + "\nrightFrontPower: " + rightFrontPower;
    }
}
