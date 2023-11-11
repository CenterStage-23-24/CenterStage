package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.TeleOp.FieldCentricDrive;

@TeleOp(name = "Tele-op")
public class Teleop extends LinearOpMode {

    private GamepadEx gamePad1;
    private FieldCentricDrive fieldCentricDrive;


    public void runOpMode() {
        telemetry.clear();

        try {
            gamePad1 = new GamepadEx(gamepad1);
            fieldCentricDrive = new FieldCentricDrive(telemetry, hardwareMap);
        } catch (Exception exception) {
            telemetry.addLine("Outside of the while loop:");
            telemetry.addLine(exception.getMessage());
            telemetry.update();
        }

        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            gamePad1.readButtons();
            fieldCentricDrive.drive(gamePad1.getLeftX(), gamePad1.getLeftY(), gamePad1.getRightX(), HWMap.readFromIMU());

        }
    }
}