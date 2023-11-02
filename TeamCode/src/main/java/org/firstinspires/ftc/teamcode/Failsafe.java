package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class Failsafe {
    // Drive Motors
    private DcMotorEx leftFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;
    private DcMotorEx rightFrontMotor;

    // Mechanism Motors
    private DcMotorEx linearSlidesRight;
    private DcMotorEx linearSlidesLeft;
    private DcMotorEx intakeMotor;
    //Servos
    private Servo outakeServoLeft;
    private Servo outakeServoRight;
    //    public Servo linearSlidesServoLeft;
//    public Servo linearSlidesServoRight;
    private static CRServo axonServoLeft;
    private static CRServo axonServoRight;
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


    public Failsafe(HWMap hardware) {
        leftBackMotor = hardware.getLeftBackMotor();
        leftFrontMotor = hardware.getLeftFrontMotor();
        rightBackMotor = hardware.getRightBackMotor();
        rightFrontMotor = hardware.getRightFrontMotor();
        linearSlidesRight = hardware.getLinearSlidesRight();
        linearSlidesLeft = hardware.getLinearSlidesLeft();
        intakeMotor = hardware.getIntakeMotor();
        outakeServoLeft = hardware.getOutakeServoLeft();
        outakeServoRight = hardware.getOutakeServoRight();
        axonAnalogLeft = hardware.getAxonAnalogLeft();
        axonAnalogRight = hardware.getAxonAnalogRight();

    }
    public void loop () {
        while (true) {

        }
    }
}
