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
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Odometry;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.Slides;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.TransferController;

@TeleOp(name = "TeleOp-1.6.0")
public class MainTeleOp extends LinearOpMode {

    public enum RobotFSM {
        start,
        cycleFSM
    }

    private RobotFSM state;
    private Cycle cycle;
    private GamepadEx gamePad1;
    private FieldCentricDrive fieldCentricDrive;
    private IntakeController intakeController;
    private Arm arm;
    private Slides slides;
    private Odometry odometry;
    private boolean backdropAlignmentAutomationStarted = false;
    private static final double ROBOT_ANGLE_ACCEPTABLE_ERROR_RANGE = 12;


    @Override
    public void runOpMode() {

        try {
            //Core
            HWMap hwMap = new HWMap(hardwareMap);
            gamePad1 = new GamepadEx(gamepad1);
            //Mechanisms
            fieldCentricDrive = new FieldCentricDrive(hwMap, telemetry);
            Intake intake = new Intake(hwMap, telemetry);
            arm = new Arm(hwMap, telemetry);
            slides = new Slides(hwMap, telemetry);
            odometry = new Odometry(hwMap);
            //Controllers
            Gripper gripper = new Gripper(hwMap);
            TransferController transferController = new TransferController(arm, slides);
            intakeController = new IntakeController(intake, gamePad1, gripper, telemetry);
            //FSMs
            cycle = new Cycle(gamePad1, telemetry, transferController, intakeController, gripper);

            //Setup
            state = RobotFSM.cycleFSM;


            telemetry.addData("INIT: ", "MainTeleOp");
            telemetry.update();
        } catch (Exception exception) {
            telemetry.addLine("Outside of the while loop:");
            telemetry.addLine(exception.getMessage());
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive()) {
            //Reading the gamepad
            odometry.retractOdo();
            gamePad1.readButtons();
            switch (state) {
                case start:
                    state = RobotFSM.cycleFSM;
                    break;
                case cycleFSM:
                    if (gamePad1.wasJustPressed(GamepadKeys.Button.X)) {
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
        //Setting alignment angle
        if (gamePad1.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            backdropAlignmentAutomationStarted = true;
            if ((HWMap.readFromIMU() >= 1) && (HWMap.readFromIMU() <= 179))
                fieldCentricDrive.setBackdropAngle(90);
            else
                fieldCentricDrive.setBackdropAngle(270);
        }

        //Backdrop Alignment Logic
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


        // Transfer logic
        intakeController.intakeControl(cycle.getToTransfer());
        slides.pid(cycle.getToTransfer());
        arm.updatePos();

        //Telemetry
        intakeController.getTelemetry();
        telemetry.update();
    }

}
