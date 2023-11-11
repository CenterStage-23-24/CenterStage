package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TeleopLinearOpMode extends LinearOpMode {

    public enum RobotFSM {
        start,
        cycleFSM,
    }

    RobotFSM state;
    Cycle cycle;
    private GamepadEx gamepad;
    DcMotorEx motor;

    @Override
    public void runOpMode() {
        try {
            motor = hardwareMap.get(DcMotorEx.class, "IM");
            gamepad = new GamepadEx(gamepad1);
            cycle = new Cycle(gamepad, telemetry, motor);
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
            //drivetrain code here
            gamepad.readButtons();
        }
    }
}

