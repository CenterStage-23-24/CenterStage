package org.firstinspires.ftc.teamcode.TeleOp.FSMs;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Gripper;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.IntakeController;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Slides;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.TransferController;

@TeleOp(name = "TeleOp")
public class MainTeleOp extends LinearOpMode {

    public enum RobotFSM {
        start,
        cycleFSM
    }

    private RobotFSM state;
    private Cycle cycle;
    private GamepadEx gamePad1;
    private GamepadEx gamePad2;
    private FieldCentricDrive fieldCentricDrive;
    private IntakeController intakeController;
    private TransferController transferController;
    private Gripper gripper;
    private Arm arm;
    private Slides slides;
    private Intake intake;

    @Override
    public void runOpMode() {

        try {
            //Core
            HWMap hwMap = new HWMap(hardwareMap);
            gamePad1 = new GamepadEx(gamepad1);
            gamePad2 = new GamepadEx(gamepad2);
            //Mechanisms
            fieldCentricDrive = new FieldCentricDrive(hwMap);
            intake = new Intake(hwMap, telemetry);
            arm = new Arm(hwMap, telemetry);
            slides = new Slides(hwMap, telemetry);
            //Controllers
            gripper = new Gripper(hwMap);
            intakeController = new IntakeController(intake, gamePad1, gripper);
            transferController = new TransferController(arm, slides, telemetry);
            //FSMs
            cycle = new Cycle(gamePad1, telemetry, transferController, gripper);

            //Setup
            state = RobotFSM.cycleFSM;
            gripper.releaseLeft();
            gripper.releaseRight();


            telemetry.addData("INIT: ", "MainTeleOp");
            telemetry.update();
        } catch (Exception exception) {
            telemetry.addLine("Outside of the while loop:");
            telemetry.addLine(exception.getMessage());
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive()) {

            gamePad1.readButtons();
            gamePad2.readButtons();
            switch (state) {
                case start:
                    telemetry.addData("in start", 1);
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
            updateCycle();
        }
    }
    private void updateCycle(){
        //LeftY is normally supposed to be negative but this is inbuilt in the gamepadEx class
        fieldCentricDrive.drive(gamePad1.getLeftX(), gamePad1.getLeftY(), gamePad1.getRightX(), HWMap.readFromIMU());
        intakeController.intakeControl(cycle.getToTransfer());
        slides.pid();
        arm.updatePos();
        telemetry.addData("Left pixel", intake.getPixelInLeft());
        telemetry.addData("Right pixel", intake.getPixelInRight());
        telemetry.update();
    }
}
