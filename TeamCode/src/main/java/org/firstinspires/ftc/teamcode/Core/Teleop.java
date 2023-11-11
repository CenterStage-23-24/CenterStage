//EVERYTHING WORKS
package org.firstinspires.ftc.teamcode.Core;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.FieldCentricFTCLib;

@TeleOp(name = "Tele-op")
public class Teleop extends LinearOpMode {
    public GamepadEx gamepadEx;
    private FieldCentricFTCLib fieldCentricDrive;
    private HWMap hwMap;


    public void runOpMode() {
        telemetry.clear();

        try {
            gamepadEx = new GamepadEx(gamepad1);
            hwMap = new HWMap(telemetry, hardwareMap);
            fieldCentricDrive = new FieldCentricFTCLib(telemetry, hardwareMap, hwMap);
        } catch (Exception exception) {
            telemetry.addLine("Outside of the while loop:");
            telemetry.addLine(exception.getMessage());
            telemetry.update();
        }

        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {


            fieldCentricDrive.drive(-gamepadEx.getLeftX(), -gamepadEx.getLeftY(), -gamepadEx.getRightX(), HWMap.readFromIMU());

        }
    }
}