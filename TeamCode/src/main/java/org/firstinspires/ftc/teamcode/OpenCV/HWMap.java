package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
    public Servo linearSlidesServoLeft;
    public Servo linearSlidesServoRight;

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
    public int servoOpen = 1;
    public int servoClose = 0;

    public ElapsedTime timer = new ElapsedTime();

    public HWMap(Telemetry telemetry, HardwareMap hardwareMap){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        //Drive Motors
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "LB"); //CH Port
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "LF"); //CH Port
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "RB"); //CH Port
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF"); //CH Port

        //Linear Slides Motors
        linearSlidesRight = hardwareMap.get(DcMotorEx.class, "LSR"); //EH Port
        linearSlidesLeft = hardwareMap.get(DcMotorEx.class, "LSL"); //EH Port

        // Intake Motor
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IM"); //EH Port

        //IMU mapped and initialized in SampleMecanumDrive - CH 12C BUS 0

        //Outake Servos
        outakeServoLeft = hardwareMap.get(Servo.class, "OSL"); //EH Port
        outakeServoRight = hardwareMap.get(Servo.class, "OSR");//EH Port

        //ODO retraction Servos
        OdoRetractionLeft = hardwareMap.get(Servo.class, "ORL"); //CH Port
        OdoRetractionRight = hardwareMap.get(Servo.class, "ORR");//CH Port
        OdoRetractionMiddle = hardwareMap.get(Servo.class, "ORM");//CH Port

        //Linear Slides Serovs
        linearSlidesServoLeft = hardwareMap.get(Servo.class, "LSSL");//EH Port
        linearSlidesServoRight = hardwareMap.get(Servo.class, "LSSR");//EH Port
        //Mapping Sensors
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "DSL");//EH Port
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "DSR");//EH Port
        trayLeftCS = hardwareMap.get(ColorSensor.class, "TLCS");//CH Port
        trayRightCS = hardwareMap.get(ColorSensor.class, "TRCS");//CH Port


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
    public void telemetry(){
        telemetry.addData("DSL: ", distanceSensorLeft.getDistance(DistanceUnit.MM));
        telemetry.addData("DSR: ", distanceSensorRight.getDistance(DistanceUnit.MM));
        telemetry.addLine(String.format("Color Sensor Left - R: %d G: %d B: %d", trayLeftCS.red(),trayLeftCS.blue(),trayLeftCS.green()));
        telemetry.addLine(String.format("Color Sensor Right - R: %d G: %d B: %d", trayRightCS.red(),trayRightCS.blue(),trayRightCS.green()));
    }
    public void open(Servo servo){
        servo.setPosition(1);
    }

    public void close(Servo servo){
        servo.setPosition(0);
    }
}
