package org.firstinspires.ftc.teamcode.Core;

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
    private DcMotorEx leftFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;
    private DcMotorEx rightFrontMotor;

    // Mechanism Motors
    private DcMotorEx linearSlidesRight;
    private DcMotorEx linearSlidesLeft;
    private DcMotorEx intakeMotor;
    //IMU
    public static BNO055IMU imu;
    private static double imuAngle;

    //Servos
    private Servo outakeServoLeft;
    private Servo outakeServoRight;
    private CRServo axonServoLeft;
    private CRServo axonServoRight;
    private AnalogInput axonAnalogLeft;
    private AnalogInput axonAnalogRight;
    private Servo OdoRetractionLeft;
    private Servo OdoRetractionRight;
    private Servo OdoRetractionMiddle;

    //Sensors

    private ColorSensor trayLeftCS;
    private ColorSensor trayRightCS;
    private DistanceSensor distanceSensorLeft;
    private DistanceSensor distanceSensorRight;

    //Other Variables
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public final double servoOpen = 1.0;
    public final double servoClose = 0.0;
    
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

    @SuppressLint("DefaultLocale")
    public void Telemetry() {
        telemetry.addData("DSL: ", getDistanceSensorLeft().getDistance(DistanceUnit.MM));
        telemetry.addData("DSR: ", getDistanceSensorRight().getDistance(DistanceUnit.MM));
        telemetry.addLine(String.format("Color Sensor Left - R: %d G: %d B: %d", getTrayLeftCS().red(), getTrayLeftCS().blue(), getTrayLeftCS().green()));
        telemetry.addLine(String.format("Color Sensor Right - R: %d G: %d B: %d", getTrayRightCS().red(), getTrayRightCS().blue(), getTrayRightCS().green()));
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

    public double voltsToDeg(AnalogInput servoEncoder){
        return (servoEncoder.getVoltage()/ 3.3 * 360);
    }

    public Servo getOdoRetractionRight() {
        return OdoRetractionRight;
    }

    public DistanceSensor getDistanceSensorRight() {
        return distanceSensorRight;
    }

    public DistanceSensor getDistanceSensorLeft() {
        return distanceSensorLeft;
    }

    public ColorSensor getTrayRightCS() {
        return trayRightCS;
    }

    public ColorSensor getTrayLeftCS() {
        return trayLeftCS;
    }

    public Servo getOdoRetractionLeft() {
        return OdoRetractionLeft;
    }

    public CRServo getAxonServoRight() {
        return axonServoRight;
    }

    public CRServo getAxonServoLeft() {
        return axonServoLeft;
    }

    public Servo getOutakeServoRight() {
        return outakeServoRight;
    }

    public Servo getOutakeServoLeft() {
        return outakeServoLeft;
    }

    public AnalogInput getAxonAnalogRight() {
        return axonAnalogRight;
    }

    public AnalogInput getAxonAnalogLeft() {
        return axonAnalogLeft;
    }

    public DcMotorEx getLinearSlidesRight() {
        return linearSlidesRight;
    }

    public DcMotorEx getLinearSlidesLeft() {
        return linearSlidesLeft;
    }

    public DcMotorEx getIntakeMotor() {
        return intakeMotor;
    }

    public DcMotorEx getRightBackMotor() {
        return rightBackMotor;
    }

    public DcMotorEx getLeftBackMotor() {
        return leftBackMotor;
    }

    public DcMotorEx getRightFrontMotor() {
        return rightFrontMotor;
    }

    public DcMotorEx getLeftFrontMotor() {
        return leftFrontMotor;
    }

    public int getOdoReadingLeft(){
        return rightBackMotor.getCurrentPosition();
    }
    public int getOdoReadingPerpendicular(){
        return leftBackMotor.getCurrentPosition();
    }
    public int getOdoReadingRight(){
        return leftFrontMotor.getCurrentPosition();
    }

}