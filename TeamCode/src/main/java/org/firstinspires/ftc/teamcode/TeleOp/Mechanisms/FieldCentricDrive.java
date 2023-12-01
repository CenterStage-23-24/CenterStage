package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;


public class FieldCentricDrive {
    private final MecanumDrive mecanumDrive;
    private final PIDController pidController;
    private static final double P = 0, I = 0, D = 0;
    private static final double BACKDROP_ANGLE = 90;
    private final Telemetry telemetry;

    public FieldCentricDrive(HWMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        mecanumDrive = hwMap.getMecanumDrive();

        pidController = new PIDController(P, I, D);
    }

    public void drive(double strafe, double forward, double turn, double heading) {
        mecanumDrive.driveFieldCentric(strafe, forward, turn, heading);
    }

    public double backdropAlignment() {
        double deltaAngle = shortestDistance(BACKDROP_ANGLE, HWMap.readFromIMU());
        double dir = dir(BACKDROP_ANGLE, HWMap.readFromIMU());
        // Distance between measured and target position * the sign of that distance
        double error = deltaAngle * dir;

        //It is 0 because we don't need to tell it what angle it is at, rather tell it how much it has to move.
        double power = pidController.calculate(HWMap.readFromIMU(), error);

        // Telemetry
        telemetry.addData("Heading: ", HWMap.readFromIMU());
        telemetry.addData("Error: ", error);
        telemetry.addData("Delta Angle: ", deltaAngle);
        telemetry.addData("Sign: ", dir);

        return power;
    }

    private double normalizeDegrees(double angle) {
        //This will take in any angle and normalize it between the range of 0-360 deg. For example, 720 deg = 0 deg and 728 deg = 8 deg
        return (angle + 360) % 360;
    }

    private double shortestDistance(double target, double position) {
        //This segment finds the shortest distance
        return Math.min(normalizeDegrees(target - position), 360 - normalizeDegrees(target - position));
    }

    // Finds the direction of the smallest distance between 2 angles
    private double dir(double position, double target) {
        return -(Math.signum(normalizeDegrees(target - position) - (360 - normalizeDegrees(target - position))));
    }

    public boolean robotAtAngle(double buffer) {
        return (((BACKDROP_ANGLE + buffer) >= HWMap.readFromIMU()) && ((BACKDROP_ANGLE - buffer) <= HWMap.readFromIMU()));
    }

}