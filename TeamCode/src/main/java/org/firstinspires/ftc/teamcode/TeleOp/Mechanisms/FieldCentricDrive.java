package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class FieldCentricDrive {
    private final MecanumDrive mecanumDrive;
    private final PIDController pidController;
    //private static final double P = 0.003, I = 0, D = 0; // Test Bench
    private static final double P = 0.003, I = 0, D = 0; //Robot
    public double backdropAngle = 90;
    private final Telemetry telemetry;


    public FieldCentricDrive(HWMap hwMap, Telemetry telemetry) {

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());;
        mecanumDrive = hwMap.getMecanumDrive();

        pidController = new PIDController(P, I, D);
    }

    public void drive(double strafe, double forward, double turn, double heading) {
        mecanumDrive.driveFieldCentric(strafe, forward, turn, heading);
    }

    public double backdropAlignment() {
        //pidController.setPID(P,I,D);

        double deltaAngle = shortestDistance(HWMap.readFromIMU());
        double dir = dir(HWMap.readFromIMU());

        // Distance between measured and target position * the sign of that distance
        double error = deltaAngle * dir;

        /*It is 0 because we already figure the error out in the deltaAngle variable
         *where we set it equal to the targetPos - measured Pos.*/
        double power = pidController.calculate(0, error);

        //This is just as a safety measure if something goes wrong inside the code.
        power = Math.abs(power) * Math.signum(power);

        // Telemetry
        telemetry.addData("Heading: ", HWMap.readFromIMU());
        telemetry.addData("Target Pos: ", backdropAngle);
        telemetry.addData("Error: ", error);
        telemetry.addData("Delta Angle: ", deltaAngle);
        telemetry.addData("Sign: ", dir);

        return power;
    }
    public double backdropAlignment(double p, double i, double d) {
        pidController.setPID(p,i,d);

        double deltaAngle = shortestDistance(HWMap.readFromIMU());
        double dir = dir(HWMap.readFromIMU());

        // Distance between measured and target position * the sign of that distance
        double error = deltaAngle * dir;

        /*It is 0 because we already figure the error out in the deltaAngle variable
         *where we set it equal to the targetPos - measured Pos.*/
        double power = pidController.calculate(0, error);

        //This is just as a safety measure if something goes wrong inside the code.
        power = Math.abs(power) * Math.signum(power);

        // Telemetry
        telemetry.addData("Heading: ", HWMap.readFromIMU());
        telemetry.addData("Target Pos: ", backdropAngle);
        telemetry.addData("Error: ", error);
        telemetry.addData("Delta Angle: ", deltaAngle);
        telemetry.addData("Sign: ", dir);

        return power;
    }

    public double normalizeDegrees(double angle) {
        //This will take in any angle and normalize it between the range of 0-360 deg. For example, 720 deg = 0 deg and 728 deg = 8 deg
        return (angle + 360) % 360;
    }

    private double shortestDistance(double position) {
        //This segment finds the shortest distance
        return Math.min(normalizeDegrees(backdropAngle - position), 360 - normalizeDegrees(backdropAngle - position));
    }

    // Finds the direction of the smallest distance between 2 angles
    private double dir(double position) {
        return (Math.signum(normalizeDegrees(backdropAngle - position) - (360 - normalizeDegrees(backdropAngle - position))));
    }

    public boolean robotAtAngle(double buffer) {
        return (((backdropAngle + buffer) >= normalizeDegrees(HWMap.readFromIMU())) && ((backdropAngle - buffer) <= normalizeDegrees(HWMap.readFromIMU())));
    }
    public boolean robotAtAngle(double alignmentAngle, double buffer) {
        return (((alignmentAngle + buffer) >= normalizeDegrees(HWMap.readFromIMU())) && ((alignmentAngle - buffer) <= normalizeDegrees(HWMap.readFromIMU())));
    }

    public void setBackdropAngle(double backdropAngle) {
        this.backdropAngle = backdropAngle;
    }
    public void setBackdropPID(double p, double i, double d){
        pidController.setPID(p,i,d);
    }

}