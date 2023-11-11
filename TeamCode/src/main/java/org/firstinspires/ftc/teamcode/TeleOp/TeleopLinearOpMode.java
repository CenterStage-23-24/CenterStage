package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.Cycle;

@TeleOp
public class TeleopLinearOpMode extends LinearOpMode {

    public enum RobotFSM {
        start,
        cycleFSM,
    }

    RobotFSM state;
    Cycle cycle;
    FieldCentricDrive fieldCentricDrive;
    private GamepadEx gamepad;

    @Override
    public void runOpMode() {
        try {
            gamepad = new GamepadEx(gamepad1);
            fieldCentricDrive = new FieldCentricDrive(telemetry, hardwareMap);
            cycle = new Cycle(gamepad, telemetry, fieldCentricDrive);
            state = RobotFSM.start;
            telemetry.addData("-", "Init Done");
            telemetry.update();
        } catch (Exception exception) {
            telemetry.addLine("Outside of the while loop:");
            telemetry.addLine(exception.getMessage());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            gamepad.readButtons();
            switch (state) { //exit state?
                case start:
                    telemetry.addData("in start", 1);
                    if (gamepad.getButton(GamepadKeys.Button.X)) {
                        telemetry.addData("x pressed", 1);
                        state = RobotFSM.cycleFSM;
                    }
                    break;
                case cycleFSM:
                    telemetry.addData("in cycle state", 1);
                    cycle.loop();
                    state = RobotFSM.start;
                    break;
            }
            telemetry.addData("end of Main loop", 1);
            telemetry.update();

            fieldCentricDrive.drive(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX(), HWMap.readFromIMU());

            gamepad.readButtons();
        }
    }
}

