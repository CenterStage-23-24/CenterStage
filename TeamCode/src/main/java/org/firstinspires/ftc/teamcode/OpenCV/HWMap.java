package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This class contains all the hardware components that are programmed on our robot and are mapped to the robot as well.
 * Other variables like Telemetry and ElapsedTime are also created.
 */

public class HWMap {
    // Drive Motors
    public DcMotorEx leftFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;
    public DcMotorEx rightFrontMotor;

    // Mechanism Motors
    public DcMotorEx linearSlidesRight;
    public DcMotorEx linearSlidesLeft;
    public DcMotorEx intakeMotor;
    //IMU
    public static BNO055IMU imu;
    public static double imuAngle;

    //Servos
    public Servo outakeServoLeft;
    public Servo outakeServoRight;
    public CRServo axonServoLeft;
    public CRServo axonServoRight;
    public AnalogInput axonAnalogLeft;
    public AnalogInput axonAnalogRight;
    public Servo OdoRetractionLeft;
    public Servo OdoRetractionRight;
    public Servo OdoRetractionMiddle;

    //Sensors

    public ColorSensor trayLeftCS;
    public ColorSensor trayRightCS;
    public DistanceSensor distanceSensorLeft;
    public DistanceSensor distanceSensorRight;

    //Other Variables
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public final double servoOpen = 1.0;
    public final double servoClose = 0.0;

    public ElapsedTime timer = new ElapsedTime();

    public HWMap(Telemetry telemetry, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;


        //Drive Motors
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF"); //CH Port 0
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "LF"); //CH Port 1. The right odo pod accesses this motor's encoder port
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "LB"); //CH Port 2. The perpendicular odo pod accesses this motor's encoder port
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "RB"); //CH Port 3. The left odo pod accesses this motor's encoder port.

        //Linear Slides Motors
        linearSlidesLeft = hardwareMap.get(DcMotorEx.class, "LSL"); //EH Port 2
        linearSlidesRight = hardwareMap.get(DcMotorEx.class, "LSR"); //EH Port 3

        // Intake Motor
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IM"); //EH Port 0

        //IMU mapped and initialized in SampleMecanumDrive - CH 12C BUS 0

        //Outake Servos
        outakeServoLeft = hardwareMap.get(Servo.class, "OSL"); //EH Port 4
        outakeServoRight = hardwareMap.get(Servo.class, "OSR");//EH Port 5

        //ODO retraction Servos
        OdoRetractionLeft = hardwareMap.get(Servo.class, "ORL"); //CH Port 0
        OdoRetractionRight = hardwareMap.get(Servo.class, "ORR");//CH Port 1
        OdoRetractionMiddle = hardwareMap.get(Servo.class, "ORM");//CH Port 2

        //Linear Slides Servos
        axonServoLeft = hardwareMap.get(CRServo.class, "ASL");//EH Port 0
        axonServoRight = hardwareMap.get(CRServo.class, "ASR");//EH Port 1
        axonAnalogLeft = hardwareMap.get(AnalogInput.class, "AAL"); //EH Port 0
        axonAnalogRight = hardwareMap.get(AnalogInput.class, "AAR"); //EH Port 2
        //Right will be CW and left will be CCW. Taped servo is CCW

        //Mapping Sensors
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "DSL");//EH Port 2
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "DSR");//EH Port 0
        trayLeftCS = hardwareMap.get(ColorSensor.class, "TLCS");//CH Port 2
        trayRightCS = hardwareMap.get(ColorSensor.class, "TRCS");//CH Port 1


        //Set Motor Direction
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        //Zero Power Behavior
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set Motor Mode
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearSlidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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


    @SuppressLint("DefaultLocale")
    public void Telemetry() {
        telemetry.addData("DSL: ", distanceSensorLeft.getDistance(DistanceUnit.MM));
        telemetry.addData("DSR: ", distanceSensorRight.getDistance(DistanceUnit.MM));
        telemetry.addLine(String.format("Color Sensor Left - R: %d G: %d B: %d", trayLeftCS.red(), trayLeftCS.blue(), trayLeftCS.green()));
        telemetry.addLine(String.format("Color Sensor Right - R: %d G: %d B: %d", trayRightCS.red(), trayRightCS.blue(), trayRightCS.green()));
        telemetry.addData("OPL: ", rightBackMotor.getCurrentPosition());
        telemetry.addData("OPF: ", leftBackMotor.getCurrentPosition());
        telemetry.addData("OPR: ", leftFrontMotor.getCurrentPosition());
        telemetry.update();
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

    public double voltsToDeg(AnalogInput servoEncoder){
        return (servoEncoder.getVoltage()/ 3.3 * 360);
    }


}