package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
    private DcMotorEx leftFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;
    private DcMotorEx rightFrontMotor;

    // Mechanism Motors
    private DcMotorEx linearSlidesRight;
    private DcMotorEx linearSlidesLeft;
    private DcMotorEx intakeMotor;
    //IMU
    private static BNO055IMU imu;
    private static double imuAngle;

    //Servos
    private Servo outakeServoLeft;
    private Servo outakeServoRight;
    //    public Servo linearSlidesServoLeft;
//    public Servo linearSlidesServoRight;
    private static AnalogInput axonLeft;
    private static AnalogInput axonRight;
    private Servo OdoRetractionLeft;
    private Servo OdoRetractionRight;
    private Servo OdoRetractionMiddle;

    //Sensors
    private ColorSensor trayLeftCS;
    private ColorSensor trayRightCS;
    private DistanceSensor distanceSensorLeft;
    private DistanceSensor distanceSensorRight;

    //Other Variables needed
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private static double servoOpen = 1.0;
    private double servoClose = 0.0;
    private static double axonPosLeft = axonLeft.getVoltage()/3.3 * 360;
    private static double axonPosRight = axonRight.getVoltage()/3.3 * 360;


    private ElapsedTime timer = new ElapsedTime();

    public HWMap(Telemetry telemetry, HardwareMap hardwareMap){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        //Drive Motors
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF"); //CH Port 0
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "LF"); //CH Port 1
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "LB"); //CH Port 2
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "RB"); //CH Port 3

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
//        linearSlidesServoLeft = hardwareMap.get(Servo.class, "LSSL");//EH Port 0
//        linearSlidesServoRight = hardwareMap.get(Servo.class, "LSSR");//EH Port 1
        axonLeft = hardwareMap.get(AnalogInput.class, "AL"); //EH Port
        axonRight = hardwareMap.get(AnalogInput.class, "AR"); //EH Port

        //Mapping Sensors
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "DSL");//EH Port 2
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "DSR");//EH Port 0
        trayLeftCS = hardwareMap.get(ColorSensor.class, "TLCS");//CH Port 2
        trayRightCS = hardwareMap.get(ColorSensor.class, "TRCS");//CH Port 0


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
    public void Telemetry(){
        telemetry.addData("DSL: ", distanceSensorLeft.getDistance(DistanceUnit.MM));
        telemetry.addData("DSR: ", distanceSensorRight.getDistance(DistanceUnit.MM));
        telemetry.addLine(String.format("Color Sensor Left - R: %d G: %d B: %d", trayLeftCS.red(),trayLeftCS.blue(),trayLeftCS.green()));
        telemetry.addLine(String.format("Color Sensor Right - R: %d G: %d B: %d", trayRightCS.red(),trayRightCS.blue(),trayRightCS.green()));
        telemetry.update();
    }
    public void open(Servo servo){
        servo.setPosition(1);
    }

    public void close(Servo servo){
        servo.setPosition(0);
    }

    public boolean isServoAtPos(Servo servo, Double Pos) {
        return servo.getPosition() == Pos;
    }

    public DcMotorEx getLeftFrontMotor() {
        return leftFrontMotor;
    }

    public DcMotorEx getLeftBackMotor() {
        return leftBackMotor;
    }

    public DcMotorEx getRightBackMotor() {
        return rightBackMotor;
    }

    public DcMotorEx getRightFrontMotor() {
        return rightFrontMotor;
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

    public static BNO055IMU getImu() {
        return imu;
    }

    public static double getImuAngle() {
        return imuAngle;
    }

    public Servo getOutakeServoLeft() {
        return outakeServoLeft;
    }

    public Servo getOutakeServoRight() {
        return outakeServoRight;
    }

    public static AnalogInput getAxonLeft() {
        return axonLeft;
    }

    public static AnalogInput getAxonRight() {
        return axonRight;
    }

    public Servo getOdoRetractionLeft() {
        return OdoRetractionLeft;
    }

    public Servo getOdoRetractionRight() {
        return OdoRetractionRight;
    }

    public Servo getOdoRetractionMiddle() {
        return OdoRetractionMiddle;
    }

    public ColorSensor getTrayLeftCS() {
        return trayLeftCS;
    }

    public ColorSensor getTrayRightCS() {
        return trayRightCS;
    }

    public DistanceSensor getDistanceSensorLeft() {
        return distanceSensorLeft;
    }

    public DistanceSensor getDistanceSensorRight() {
        return distanceSensorRight;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public static double getServoOpen() {
        return servoOpen;
    }

    public double getServoClose() {
        return servoClose;
    }

    public static double getAxonPosLeft() {
        return axonPosLeft;
    }

    public static double getAxonPosRight() {
        return axonPosRight;
    }

    public ElapsedTime getTimer() {
        return timer;
    }
}