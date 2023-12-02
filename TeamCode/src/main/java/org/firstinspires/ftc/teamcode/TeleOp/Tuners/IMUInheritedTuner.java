package org.firstinspires.ftc.teamcode.TeleOp.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;

@Config
public class IMUInheritedTuner extends FieldCentricDrive {
    private PIDController pidController;
    private Telemetry telemetry;
    private MecanumDrive mecanumDrive;
    public static double BACKDROP_ANGLE = 270;
    public static double p = 0.003, i = 0.0, d = 0.0;

    public IMUInheritedTuner(HWMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
        this.telemetry = telemetry;
        pidController = new PIDController(p,i,d);
        mecanumDrive = hwMap.getMecanumDrive();
    }

    public void drive(double strafe, double forward, double turn, double heading) {
        mecanumDrive.driveFieldCentric(strafe, forward, turn, heading);
    }


    public double backdropAlignment() {
        pidController.setPID(p,i,d);
        double normalizeHeading = normalizeDegrees(HWMap.readFromIMU());

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
        telemetry.addData("P: ", p);
        telemetry.addData("I: ", i);
        telemetry.addData("D: ", d);
        telemetry.addData("Heading: ", normalizeHeading);
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
        return Math.min(normalizeDegrees(BACKDROP_ANGLE - position), 360 - normalizeDegrees(BACKDROP_ANGLE - position));
    }

    // Finds the direction of the smallest distance between 2 angles
    private double dir(double position) {
        return (Math.signum(normalizeDegrees(BACKDROP_ANGLE - position) - (360 - normalizeDegrees(BACKDROP_ANGLE - position))));
    }

    public boolean robotAtAngle(double buffer) {
        return (((BACKDROP_ANGLE + buffer) >= HWMap.readFromIMU()) && ((BACKDROP_ANGLE - buffer) <= HWMap.readFromIMU()));
    }
}
