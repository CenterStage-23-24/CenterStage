package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.HWMapFTCLib;

@TeleOp(name = "Tele-op")
public class Teleop extends LinearOpMode {

    private Controller controller;
    private FieldCentricFTCLib fieldCentricDrive;


    public void runOpMode() {
        telemetry.clear();

        try {
            controller = new Controller();
            fieldCentricDrive = new FieldCentricFTCLib(telemetry, hardwareMap);
        } catch (Exception exception) {
            telemetry.addLine("Outside of the while loop:");
            telemetry.addLine(exception.getMessage());
            telemetry.update();
        }

        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            controller.update();


            //FIELD-CENTERIC_______________________________________________________________________________
            double gamepadX;
            double gamepadY;
            double gamepadRot;

            if (Math.abs(controller.gamepad1X) > 0.01) {
                gamepadX = controller.gamepad1X;
            } else if (Math.abs(controller.gamepad2X) > 0.01) {
                gamepadX = controller.gamepad2X;
            } else {
                gamepadX = 0;
            }
            if (Math.abs(controller.gamepad1Y) > 0.01) {
                gamepadY = controller.gamepad1Y;
            } else if (Math.abs(controller.gamepad2Y) > 0.01) {
                gamepadY = controller.gamepad2Y;
            } else {
                gamepadY = 0;
            }
            if (Math.abs(controller.gamepad1Rot) > 0.01) {
                gamepadRot = -controller.gamepad1Rot;
            } else if (Math.abs(controller.gamepad2Rot) > 0.01) {
                gamepadRot = -controller.gamepad2Rot;
            } else {
                gamepadRot = 0;
            }

            fieldCentricDrive.drive(gamepadX, gamepadY, gamepadRot, HWMapFTCLib.readFromIMU());

        }
    }
}