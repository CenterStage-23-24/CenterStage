package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.HWMap;

public class FieldCentricDrive extends HWMap {

    private HWMap hwMap;
    private DcMotorEx leftFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx rightBackMotor;


    public double imuMeasure;
    public double leftBackPower;
    public double rightBackPower;
    public double rightFrontPower;
    public double leftFrontPower;
    public double rotationEffectivness = 0.7;
    public double xyEffectivness = 0.9;
    private ElapsedTime loopTimer = new ElapsedTime();


    public FieldCentricDrive(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);
        hwMap = new HWMap(telemetry, hardwareMap);
        leftBackMotor = hwMap.getLeftBackMotor();
        leftFrontMotor = hwMap.getLeftFrontMotor();
        rightBackMotor = hwMap.getRightBackMotor();
        rightFrontMotor = hwMap.getRightFrontMotor();
        hwMap.initializeIMU();
    }

    public void drive(double gamepadX, double gamepadY, double gamepadRot) {

        // gamepadRot is negated because in math, a counterclockwise rotation is positive
        // (think unit circle), but on the controller, we expect the robot to rotate clockwise when
        // we push the stick to the right. Pushing the stick to the right outputs a positive value.
        double turn = -gamepadRot * rotationEffectivness;
        double controllerX = gamepadX * xyEffectivness;
        double controllerY = gamepadY * xyEffectivness;
        double[] controllerVector = {controllerX, controllerY};

        imuMeasure = readFromIMU();

        double[] rotatedVector = rotate(controllerVector, imuMeasure);
        double rotatedX = rotatedVector[0];
        double rotatedY = rotatedVector[1];

        double theta = Math.atan2(rotatedY, rotatedX);
        double power = Math.hypot(rotatedX, rotatedY);
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        leftBackPower = power * sin / max + turn;
        leftFrontPower = power * cos / max + turn;
        rightBackPower = power * cos / max - turn;
        rightFrontPower = power * sin / max - turn;

        if ((power + Math.abs(turn)) > 1) {
            leftFrontPower /= power + Math.abs(turn);
            rightFrontPower /= power + Math.abs(turn);
            leftBackPower /= power + Math.abs(turn);
            rightBackPower /= power + Math.abs(turn);
        }

        leftBackMotor.setPower(leftBackPower);
        leftFrontMotor.setPower(leftFrontPower);
        rightBackMotor.setPower(rightBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        loopTimer.reset();
    }

    public static double[] rotate(double[] vector, double angle) {
        final double[] newVector = {0, 0};
        newVector[0] = Math.cos(angle) * vector[0] + (-Math.sin(angle)) * vector[1];
        newVector[1] = Math.sin(angle) * vector[0] + Math.cos(angle) * vector[1];
        return newVector;
    }
}