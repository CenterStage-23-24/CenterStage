package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "controllerTest1")

public class controllerTest1 extends LinearOpMode {

    GamepadEx gamePad;

    @Override
    public void runOpMode(){

        gamePad = new GamepadEx(gamepad1);

        while(opModeIsActive()){
            gamePad.readButtons();

            //ABXY buttons
            if (gamePad.wasJustPressed(GamepadKeys.Button.B)) {
                telemetry.addLine("B Pressed");
            }
            if (gamePad.wasJustPressed(GamepadKeys.Button.A)) {
                telemetry.addLine("A Pressed");
            }
            if (gamePad.wasJustPressed(GamepadKeys.Button.Y)) {
                telemetry.addLine("Y Pressed");
            }
            if (gamePad.wasJustPressed(GamepadKeys.Button.X)) {
                telemetry.addLine("X Pressed");
            }

            //Dpad
            if (gamePad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                telemetry.addLine("Dpad_Down Pressed");
            }
            if (gamePad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                telemetry.addLine("Dpad_Up Pressed");
            }
            if (gamePad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                telemetry.addLine("Dpad_Right Pressed");
            }
            if (gamePad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                telemetry.addLine("Dpad_left Pressed");
            }

            //Bumpers
            if (gamePad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                telemetry.addLine("Left-Bumper Pressed");
            }

            if (gamePad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                telemetry.addLine("Right-Bumper Pressed");
            }

            //Triggers
            //if (gamePad.isDown(GamepadKeys.Trigger.LEFT_TRIGGER)) {
            //    telemetry.addLine("Left-Trigger Pressed");
            //}
            //if (gamePad.isDown(GamepadKeys.Trigger.RIGHT_TRIGGER)) {
            //    telemetry.addLine("Right-Trigger Pressed");
            //}
        }

    }

}
