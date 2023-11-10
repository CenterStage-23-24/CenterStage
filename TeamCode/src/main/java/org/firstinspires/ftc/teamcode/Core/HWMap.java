package org.firstinspires.ftc.teamcode.Core;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;

/**
 * This class contains all the hardware components that are programmed on our robot and are mapped to the robot as well.
 * Other variables like Telemetry and ElapsedTime are also created.
 */

public class HWMap {

    private Motor leftFrontMotor;
    private Motor leftBackMotor;
    private Motor rightBackMotor;
    private Motor rightFrontMotor;

    //IMU
    private static IMU imu;
    private static double imuAngle;

    private final Telemetry telemetry;

    public final double servoOpen = 1.0;
    public final double servoClose = 0.0;

    public HWMap(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        rightFrontMotor = new Motor(hardwareMap, "RF", Motor.GoBILDA.RPM_435); //CH Port 0
        leftFrontMotor = new Motor(hardwareMap, "LF", Motor.GoBILDA.RPM_435);//CH Port 1. The right odo pod accesses this motor's encoder port
        leftBackMotor = new Motor(hardwareMap, "LB", Motor.GoBILDA.RPM_435); //CH Port 2. The perpendicular odo pod accesses this motor's encoder port
        rightBackMotor = new Motor(hardwareMap, "RB", Motor.GoBILDA.RPM_435);//CH Port 3. The left odo pod accesses this motor's encoder port.

        //IMU mapped and initialized in SampleMecanumDrive - CH 12C BUS 0
        imu = hardwareMap.get(IMU.class, "imu");
        initializeIMU();

        //Set Motor Direction
        leftFrontMotor.setInverted(true);
        leftBackMotor.setInverted(true);
        rightFrontMotor.setInverted(true);
        rightBackMotor.setInverted(true);

        //Zero Power Behavior
        leftBackMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //Set Motor Mode
        leftBackMotor.setRunMode(Motor.RunMode.RawPower);
        rightBackMotor.setRunMode(Motor.RunMode.RawPower);
        leftFrontMotor.setRunMode(Motor.RunMode.RawPower);
        rightFrontMotor.setRunMode(Motor.RunMode.RawPower);

    }

    @SuppressLint("DefaultLocale")
    public void Telemetry() {
        telemetry.addData("OPL: ", getOdoReadingLeft());
        telemetry.addData("OPP: ", getOdoReadingPerpendicular());
        telemetry.addData("OPR: ", getOdoReadingRight());
        telemetry.update();
    }

    public static double readFromIMU() {
        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return imuAngle;
    }

    public static void initializeIMU() {
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);
    }

    public void open(Servo servo) {
        servo.setPosition(servoOpen);
    }

    public void close(Servo servo) {
        servo.setPosition(servoClose);
    }

    public void loop() {
        Telemetry();
    }

    public int voltsToDeg(AnalogInput servoEncoder) {
        return (int) (servoEncoder.getVoltage() / 3.3 * 360);
    }

    public Motor getRightBackMotor() {
        return rightBackMotor;
    }

    public Motor getLeftBackMotor() {
        return leftBackMotor;
    }

    public Motor getRightFrontMotor() {
        return rightFrontMotor;
    }

    public Motor getLeftFrontMotor() {
        return leftFrontMotor;
    }

    public int getOdoReadingLeft() {
        return rightBackMotor.getCurrentPosition();
    }

    public int getOdoReadingPerpendicular() {
        return leftBackMotor.getCurrentPosition();
    }

    public int getOdoReadingRight() {
        return leftFrontMotor.getCurrentPosition();
    }

}
