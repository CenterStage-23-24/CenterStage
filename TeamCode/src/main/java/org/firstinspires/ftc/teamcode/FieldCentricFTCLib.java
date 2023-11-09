package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class FieldCentricFTCLib extends HWMap {
    private HWMap hwMap;
    private Motor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    private MecanumDrive mecanumDrive;

    public FieldCentricFTCLib(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);
        hwMap = new HWMap(telemetry, hardwareMap);
        leftBackMotor = hwMap.getLeftBackMotor();
        leftFrontMotor = hwMap.getLeftFrontMotor();
        rightBackMotor = hwMap.getRightBackMotor();
        rightFrontMotor = hwMap.getRightFrontMotor();
        mecanumDrive = new MecanumDrive(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
    }

    public void drive(GamepadEx gamepad) {
        //FIELD-CENTERIC_______________________________________________________________________________
        double gamepadX;
        double gamepadY;
        double gamepadRot;

        if (Math.abs(gamepad.getLeftX()) > 0.01) {
            gamepadX = gamepad.getLeftX();
        } //else if (Math.abs(gamePad2.gamepad2X) > 0.01) {
        //gamepadX = controller.gamepad2X; }
        else{
            gamepadX = 0;
        }
        if (Math.abs(gamepad.getLeftY()) > 0.01) {
            gamepadY = gamepad.getLeftY();
        } // else if (Math.abs(gamePad1.gamepad2Y) > 0.01) {
        //gamepadY = gamePad1.gamepad2Y; }
        else {
            gamepadY = 0;
        }
        if (Math.abs(gamepad.getRightX()) > 0.01) {
            gamepadRot = -gamepad.getRightX();
        } //else if (Math.abs(controller.gamepad2Rot) > 0.01) {
        //gamepadRot = -controller.gamepad2Rot; }
        else {
            gamepadRot = 0;
        }

        mecanumDrive.driveFieldCentric(gamepadX, gamepadY, gamepadRot, hwMap.readFromIMU());
    }
}
