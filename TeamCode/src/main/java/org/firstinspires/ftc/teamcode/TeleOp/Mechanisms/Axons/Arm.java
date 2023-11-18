package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;

@Config
public class Arm {
    // Basic PID coefficients
    // a is a feedforward coefficient used to counteract the torque applied to the arm by gravity
    // a * sin(θ) gives the power needed to counteract gravity, θ is distance from vertically pointing down
    // a is used in place of m * g * r as those remain constant, and its easier to tune a single coefficient
    public static double p = 0.00518, i = 0.003, d = 0.015, a = 0.07;
    private final AxonClass leftAxon;
    private final AxonClass rightAxon;
    private final PIDController pidController;
    private final Telemetry telemetry;

    public Arm(HWMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        final CRServo leftServo = hwMap.getAxonServoLeft();
        final CRServo rightServo = hwMap.getAxonServoRight();
        final AnalogInput leftEncoder = hwMap.getAxonAnalogLeft();
        final AnalogInput rightEncoder = hwMap.getAxonAnalogRight();
        leftAxon = new AxonClass(leftServo, leftEncoder, true, true);
        rightAxon = new AxonClass(rightServo, rightEncoder, false, false);
        pidController = new PIDController(p, i, d);
    }


    public static final double intakePos = 115; // Angle for Intaking pixels
    public final double depositPos = normalizeRadiansTau(intakePos + 150); // Angle for depositing pixels, is 150 degrees from intake
    public static double intakeOffset = 60; // Degrees that the intake position is from vertically facing down
    private final double safeError = 10; // Position can be +- this many degrees from target for safe transfer
    private final double safeIntake = normalizeRadiansTau(intakePos - safeError); // Safe range to start transfer from intake pos
    private final double safeRange = 150 + (2 * safeError); // the range of the safe values from safeIntake, will end at depositPos + safeError

    public static double targetPos = intakePos;
    private double measuredPos = 0;

    //Temp Vars for testing

    // Finds the smallest distance between 2 angles, input and output in degrees
    private double angleDelta(double angle1, double angle2) {
        return Math.min(normalizeRadiansTau(angle1 - angle2), 360 - normalizeRadiansTau(angle1 - angle2));
    }

    // Finds the direction of the smallest distance between 2 angles
    private double angleDeltaSign(double position, double target) {
        return -(Math.signum(normalizeRadiansTau(target - position) - (360 - normalizeRadiansTau(target - position))));
    }

    // Converts angle from degrees to radians
    private double toRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    // Takes input angle in degrees, returns that angle in the range of 0-360
    private double normalizeRadiansTau(double angle) {
        return (angle + 360) % 360;
    }

    public void goToDeposit() {
        targetPos = depositPos;
    }

    public void goToIntake() {
        targetPos = intakePos;
    }

    public void updatePos() {
        measuredPos = leftAxon.getPos();

        //This is the error between measured position and target position.
        double delta = angleDelta(measuredPos, targetPos);
        double sign = angleDeltaSign(measuredPos, targetPos);
        // Distance between measured and target position * the sign of that distance
        double error = delta * sign;

        // We use zero here because we calculate the error and its direction for the PID loop
        double power = pidController.calculate(0, error);

        // Feedforward using a
        double degreesFromVert = angleDelta(measuredPos, intakePos) * angleDeltaSign(measuredPos, intakePos) + intakeOffset; // Degrees the arm is away from vertically straight down
        double feedForward = -a * Math.sin(toRadians(degreesFromVert)); // Calculating power
        power += feedForward; // Adding feedforward to power

        //Applying the sign to the power
        power = Math.abs(power) * Math.signum(power);

        // Setting servo powers, one servo should have a true value for inverse when its created so we can set positive powers to both
        leftAxon.setPower(power);
        rightAxon.setPower(power);

        // Telemetry
        telemetry.addData("Measured Pos: ", measuredPos);
        telemetry.addData("Target Pos: ", targetPos);
        telemetry.addData("Delta: ", delta);
        telemetry.addData("Sign: ", sign);
    }
    //This method returns TRUE if the axons are within the buffered range of the target position or it will return FALSE.
    public boolean axonAtPos(double targetPos, double buffer) {
        return (((targetPos + buffer) >= measuredPos) && ((targetPos - buffer) <= measuredPos));
    }

}
