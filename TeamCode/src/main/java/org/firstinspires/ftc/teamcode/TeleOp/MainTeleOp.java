package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleOp.FSMs.Cycle;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;

@TeleOp(name = "TeleOp")
public class MainTeleOp extends LinearOpMode {

    public enum RobotFSM {
        start,
        cycleFSM
    }

    private RobotFSM state;
    private HWMap hwMap;
    private Cycle cycle;
    private GamepadEx gamePad1;
    private GamepadEx gamePad2;
    private FieldCentricDrive fieldCentricDrive;


    //Hardware
    private Servo outakeServoLeft;
    private Servo outakeServoRight;
    private double openPos = 0.5;

    @Override
    public void runOpMode() {


        try {
            fieldCentricDrive = new FieldCentricDrive(telemetry, hardwareMap);
            hwMap = new HWMap(telemetry, hardwareMap);
            gamePad1 = new GamepadEx(gamepad1);
            gamePad2 = new GamepadEx(gamepad2);

            cycle = new Cycle(hwMap, hardwareMap, gamePad1, telemetry, fieldCentricDrive);
            state = RobotFSM.cycleFSM;

            outakeServoLeft = hwMap.getOutakeServoLeft();
            outakeServoRight = hwMap.getOutakeServoRight();
        } catch (Exception exception) {
            telemetry.addLine("Outside of the while loop:");
            telemetry.addLine(exception.getMessage());
            telemetry.update();
        }
        outakeServoLeft.setPosition(openPos);
        outakeServoRight.setPosition(openPos);
        waitForStart();

        while (opModeIsActive()) {
            gamePad1.readButtons();
            gamePad2.readButtons();
            switch (state) { //exit state?
                case start:
                    telemetry.addData("in start", 1);
                    if(!gamePad1.isDown(GamepadKeys.Button.DPAD_UP))
                        state = RobotFSM.cycleFSM;
                    break;
                case cycleFSM:
                    telemetry.addData("in cycle state", 1);
                    if (gamePad1.wasJustPressed(GamepadKeys.Button.B)) {
                        telemetry.addData("-", "Quit Cycle State");
                        state = RobotFSM.start;
                    } else {
                        cycle.loop();
                    }
                    break;
            }
            telemetry.update();
            fieldCentricDrive.drive(gamePad1.getLeftX(), gamePad1.getLeftY(), gamePad1.getRightX(), HWMap.readFromIMU());
            //LeftY is normally supposed to be negative but in the gamepadEx class they do that for us, but leftY is negated.
        }
    }
}
