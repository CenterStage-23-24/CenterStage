package org.firstinspires.ftc.teamcode.TeleOp.Tuners;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.AxonClass;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
@Config
public class InheritedArmTuner extends Arm {
    private HWMap hwMap;
    private CRServo leftServo;
    private CRServo rightServo;
    private AnalogInput leftEncoder;
    private AnalogInput rightEncoder;
    private AxonClass leftAxon;
    private AxonClass rightAxon;
    private PIDController pidController;
    private Telemetry telemetry;

    public InheritedArmTuner(HWMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);
        this.hwMap = hwMap;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftServo = hwMap.getAxonServoLeft();
        rightServo = hwMap.getAxonServoRight();
        leftEncoder = hwMap.getAxonAnalogLeft();
        rightEncoder = hwMap.getAxonAnalogRight();
        pidController = new PIDController(p, i, d);
        leftAxon = new AxonClass(leftServo, leftEncoder, true, true);
        rightAxon = new AxonClass(rightServo, rightEncoder, false, false);

    }

    // All code is written on the assumption that positive power -> increased position magnitude
    // and that to get from intakePos to depositPos, you need to increase your position

    // Basic PID coefficients
    public static double p = 0.006, i = 0.003, d = 0.015;

    // a is a feedforward coefficient used to counteract the torque applied to the arm by gravity
    // a * sin(θ) gives the power needed to counteract gravity, θ is distance from vertically pointing down
    // a is used in place of m * g * r as those remain constant, and its easier to tune a single coefficient
    public static double a = 0.07;

    public static final double intakePos = 115; // Angle for Intaking pixels
    public final double depositPos = super.normalizeRadiansTau(intakePos + 150); // Angle for depositing pixels, is 150 degrees from intake
    public static double intakeOffset = 60; // Degrees that the intake position is from vertically facing down
    public final double safeError = 10; // Position can be +- this many degrees from target for safe transfer
    public final double safeIntake = super.normalizeRadiansTau(intakePos - safeError); // Safe range to start transfer from intake pos
    public final double safeRange = 150 + (2 * safeError); // the range of the safe values from safeIntake, will end at depositPos + safeError

    public static double targetPos = intakePos;

    //Temp Vars for testing
    public static double posVar = 0;
    public static double powerCap = 1.0;

        public void loop() {

            // Jank code to use FTC dashboard for a boolean, 0 = intake pos, 1 = deposit pos
            if (posVar == 0){
                targetPos = intakePos;
            } else {
                targetPos = depositPos;
            }

            pidController.setPID(p, i, d);
            double measuredPos = leftAxon.getPos();

            double delta  = super.angleDelta(measuredPos, targetPos);
            double sign = super.angleDeltaSign(measuredPos, targetPos);
            // Distance between measured and target position * the sign of that distance
            double error = delta * sign;

            // We use zero here because we calculate the error and its direction for the PID loop
            double power = pidController.calculate(0, error);

            // Feedforward using a
            double degreesFromVert = super.angleDelta(measuredPos, intakePos) *  super.angleDeltaSign(measuredPos, intakePos) + intakeOffset; // Degrees the arm is away from vertically straight down
            double feedForward = -a * Math.sin(super.toRadians(degreesFromVert)); // Calculating power
            power += feedForward; // Adding feedforward to power

            // Capping the power
            power = Math.min(Math.abs(power), powerCap) * Math.signum(power);

            // Setting servo powers, one servo should have a true value for inverse when its created so we can set positive powers to both
            leftAxon.setPower(power);
            rightAxon.setPower(power);

            telemetry.addData("Power: ", power);
            telemetry.addData("Measured Pos Left: ", measuredPos);
            telemetry.addData("Measured Pos Right: ", rightAxon.getPos());
            telemetry.addData("Target Pos: ", targetPos);
            telemetry.addData("Delta: ", delta);
            telemetry.addData("Sign: ", sign);
            telemetry.addData("Error: ", error);
            telemetry.addData("Deg from vert: ", degreesFromVert);
            telemetry.update();
        }

    // Finds the smallest distance between 2 angles, input and output in degrees
    //public double angleDelta(double angle1, double angle2) {
      //  return Math.min(normalizeRadiansTau(angle1 - angle2), 360 - normalizeRadiansTau(angle1 - angle2));
   // }
    // Finds the direction of the smallest distance between 2 angles
   //public double angleDeltaSign(double position, double target) {
     //  return -(Math.signum(normalizeRadiansTau(target - position) - (360 - normalizeRadiansTau(target - position))));
    //}
    // Converts angle from degrees to radians
    //public double toRadians(double degrees) {
      //  return degrees * Math.PI/180;
    //}
    // Takes input angle in degrees, returns that angle in the range of 0-360
  //  public double normalizeRadiansTau(double angle){
    //    return (angle + 360) % 360;
    //}





}
