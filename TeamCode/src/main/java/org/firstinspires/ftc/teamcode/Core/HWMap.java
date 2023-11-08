package org.firstinspires.ftc.teamcode.Core;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;

/**
 * This class contains all the hardware components that are programmed on our robot and are mapped to the robot as well.
 * Other variables like Telemetry and ElapsedTime are also created.
 */

public class HWMap {
    // USE FOR INIT TYPE 3
    private DcMotorEx leftFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;
    private DcMotorEx rightFrontMotor;
    
    /*
    USE FOR INIT TYPE 1 + 2
    private Motor leftFrontMotor;
    private Motor leftBackMotor;
    private Motor rightBackMotor;
    private Motor rightFrontMotor;
    */
    
    //IMU
    private static BNO055IMU imu;
    private static double imuAngle;

    private final Telemetry telemetry;

    public final double servoOpen = 1.0;
    public final double servoClose = 0.0;

    public HWMap(Telemetry telemetry, HardwareMap hardwareMap) {
        //Other Variables
        this.telemetry = telemetry;
        
        //Drive Motor Init Type 3 - TEST FIRST
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF"); //CH Port 0
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "LF"); //CH Port 1. The right odo pod accesses this motor's encoder port
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "LB"); //CH Port 2. The perpendicular odo pod accesses this motor's encoder port
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "RB"); //CH Port 3. The left odo pod accesses this motor's encoder port.
        
        /*
        Drive Motor Init Type 1
        Test Results: Threw error with LF Motor
        rightFrontMotor = new Motor(hardwareMap, "RF", Motor.GoBILDA.RPM_435); //CH Port 0
        leftFrontMotor = new Motor(hardwareMap, "LF", Motor.GoBILDA.RPM_435);//CH Port 1. The right odo pod accesses this motor's encoder port
        leftBackMotor = new Motor(hardwareMap, "LB", Motor.GoBILDA.RPM_435); //CH Port 2. The perpendicular odo pod accesses this motor's encoder port
        rightBackMotor = new Motor(hardwareMap, "RB", Motor.GoBILDA.RPM_435);//CH Port 3. The left odo pod accesses this motor's encoder port.
        */

        /*
        Drive Motor Init Type 2
        Test Results: Threw error with RF Motor
        rightFrontMotor = hardwareMap.get(Motor.class, "RF"); //CH Port 0
        leftFrontMotor = hardwareMap.get(Motor.class, "LF"); //CH Port 1. The right odo pod accesses this motor's encoder port
        leftBackMotor = hardwareMap.get(Motor.class, "LB"); //CH Port 2. The perpendicular odo pod accesses this motor's encoder port
        rightBackMotor = hardwareMap.get(Motor.class, "RB"); //CH Port 3. The left odo pod accesses this motor's encoder port.
        */

        //IMU mapped and initialized in SampleMecanumDrive - CH 12C BUS 0
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initializeIMU();

        //Set Motor Direction
        leftFrontMotor.setInverted(true);
        leftBackMotor.setInverted(true);

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
        //telemetry.addData("DSL: ", getDistanceSensorLeft().getDistance(DistanceUnit.MM));
        //telemetry.addData("DSR: ", getDistanceSensorRight().getDistance(DistanceUnit.MM));
        ///telemetry.addLine(String.format("Color Sensor Left - R: %d G: %d B: %d", getTrayLeftCS().red(), getTrayLeftCS().blue(), getTrayLeftCS().green()));
        //telemetry.addLine(String.format("Color Sensor Right - R: %d G: %d B: %d", getTrayRightCS().red(), getTrayRightCS().blue(), getTrayRightCS().green()));
        telemetry.addData("OPL: ", getOdoReadingLeft());
        telemetry.addData("OPP: ", getOdoReadingPerpendicular());
        telemetry.addData("OPR: ", getOdoReadingRight());
        telemetry.update();
    }

    public static double readFromIMU() {
        imuAngle = -imu.getAngularOrientation().firstAngle;
        return imuAngle;
    }

    public static void initializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
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
