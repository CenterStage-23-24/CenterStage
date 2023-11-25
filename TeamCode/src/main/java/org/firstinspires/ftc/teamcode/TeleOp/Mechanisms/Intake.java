package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.security.Provider;

public class Intake {

    private double ejectSpeed = -0.4;
    private double leftGripPixelPos = 0;
    private double rightGripPixelPos = 1;
    private double intakeSpeed = 1.0;

    private HWMap hwMap;
    private Telemetry telemetry;

    private Motor intakeMotor;
    private Servo outtakeServoLeft;
    private Servo outtakeServoRight;
    private RevColorSensorV3 trayLeftCS;
    private RevColorSensorV3 trayRightCS;
    private final int distance = 28;
    private final int greenThreshold = 300;

    public boolean pixelInLeft;
    public boolean pixelInRight;

    public Intake(HWMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        intakeMotor = hwMap.getIntakeMotor();
        outtakeServoLeft = hwMap.getOuttakeServoLeft();
        outtakeServoRight = hwMap.getOuttakeServoRight();
        trayLeftCS = hwMap.getTrayLeftCS();
        trayRightCS = hwMap.getTrayRightCS();

    }

    public void intake() {
        intakeMotor.set(intakeSpeed);
    }

    public void intake(int motorPower){
        intakeMotor.set(motorPower);
    }

    public void eject() {
        intakeMotor.set(ejectSpeed);
    }

    public void gripLeft() {
        outtakeServoLeft.setPosition(leftGripPixelPos);
    }

    public void gripRight() {
        outtakeServoRight.setPosition(rightGripPixelPos);
    }

    public void detectPixels() {
        double csLeftDistance = trayLeftCS.getDistance(DistanceUnit.MM);
        double csRightDistance = trayRightCS.getDistance(DistanceUnit.MM);

        int leftRed = trayLeftCS.red();
        int leftBlue = trayLeftCS.blue();
        int leftGreen = trayLeftCS.green();
        int rightRed = trayRightCS.red();
        int rightBlue = trayRightCS.blue();
        int rightGreen = trayRightCS.green();

        if (csLeftDistance <= distance) {
            checkColor(leftRed, leftGreen, leftBlue);
            pixelInLeft = true;
        } else {
            telemetry.addData("-", "Nothing in the left compartment");
            pixelInLeft = false;
        }
        if (csRightDistance <= distance) {
            checkColor(rightRed, rightGreen, rightBlue);
            pixelInRight = true;
        } else {
            telemetry.addData("-", "Nothing in right compartment");
            pixelInRight = false;
        }

    }

    private void checkColor(int red, int green, int blue){
        if (green > (red * 2)) {
            telemetry.addData("-", "Green pixel detected in the right compartment");
        } else if ((blue < red) && (blue < green)) {
            telemetry.addData("-", "Yellow pixel detected in the right compartment");
        } else if ((blue > red) && (blue > green)) {
            telemetry.addData("-", "Purple pixel detected in the right compartment");
        } else if (green >= greenThreshold) {
            telemetry.addData("-", "White pixel detected in the right compartment");
        }
    }

    public boolean getPixelInLeft(){
        return pixelInLeft;
    }

    public boolean getPixelInRight(){
        return pixelInRight;
    }
}
