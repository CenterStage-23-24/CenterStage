package org.firstinspires.ftc.teamcode.TeleOp.FSMs;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Axons.Arm;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Gripper;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.IntakeController;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Slides;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.TransferController;

@TeleOp(name = "TeleOp-1.5.1")
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
    private boolean backdropAlignmentAutomationStarted = false;
    private static final double ROBOT_ANGLE_ACCEPTABLE_ERROR_RANGE = 12;


    @Override
    public void runOpMode() {

        try {
            //Core
            HWMap hwMap = new HWMap(hardwareMap);
            gamePad1 = new GamepadEx(gamepad1);
            gamePad2 = new GamepadEx(gamepad2);
            //Mechanisms
            fieldCentricDrive = new FieldCentricDrive(hwMap, telemetry);
            intake = new Intake(hwMap, telemetry);
            arm = new Arm(hwMap, telemetry);
            slides = new Slides(hwMap, telemetry);
            //Controllers
            gripper = new Gripper(hwMap);
            intakeController = new IntakeController(intake, gamePad1, gripper);
            transferController = new TransferController(arm, slides, telemetry);
            //FSMs
            cycle = new Cycle(gamePad1, telemetry, transferController,intakeController,gripper);

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

    private void updateCycle() {
        if (gamePad1.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            backdropAlignmentAutomationStarted = true;

            if ((HWMap.readFromIMU() >= 1) && (HWMap.readFromIMU() <= 179))
                fieldCentricDrive.setBackdropAngle(90);
            else
                fieldCentricDrive.setBackdropAngle(270);
        }

        if (backdropAlignmentAutomationStarted && !fieldCentricDrive.robotAtAngle(ROBOT_ANGLE_ACCEPTABLE_ERROR_RANGE)) {
            double rightX = fieldCentricDrive.backdropAlignment();
            fieldCentricDrive.drive(gamePad1.getLeftX(), gamePad1.getLeftY(), rightX, HWMap.readFromIMU());
            if ((gamePad1.getRightX() >= 0.2 || gamePad1.getRightX() <= -0.2)) {
                backdropAlignmentAutomationStarted = false;
            }
        } else {
            backdropAlignmentAutomationStarted = false;
            fieldCentricDrive.drive(gamePad1.getLeftX(), gamePad1.getLeftY(), gamePad1.getRightX(), HWMap.readFromIMU());
        }


        intakeController.intakeControl(cycle.getToTransfer());
        slides.pid(cycle.getToTransfer());
        arm.updatePos();

        telemetry.addData("toTransfer?", cycle.getToTransfer());
        telemetry.addData("Left pixel", intake.getPixelInLeft());
        telemetry.addData("Right pixel", intake.getPixelInRight());
        telemetry.addData("Intake Velocity",intake.getIntakeVelocity());
        telemetry.addData("Power Ejecting",intakeController.isPowerEjecting());
        telemetry.addData("Intake Jammed",intake.intakeJammed());
        telemetry.addData("Ramp up",intakeController.isRampUp());
        telemetry.addData("Intake Running",intakeController.isIntakeRunning());
        telemetry.addData("Jamming disabled",intakeController.isJammingDisabled());

        telemetry.addData("SLIDE TARGET POS?", slides.mmToTicks(24));
        telemetry.addData("SLIDES AT POS?", slides.atPos());
        telemetry.addData("PREV DPAD UP", cycle.getPrevUp());
        telemetry.addData("PREV DPAD DOWN", cycle.getPrevDown());
        telemetry.update();
    }

}
