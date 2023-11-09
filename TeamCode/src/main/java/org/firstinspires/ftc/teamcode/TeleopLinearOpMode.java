package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TeleopLinearOpMode extends LinearOpMode {

    public enum RobotFSM {
        start,
        cycleFSM,
        hangingFSM,
        droneFSM,
        failsafe
    }

    RobotFSM state;
    HWMap hardware;
    Cycle cycle;
    //DroneLaunch drone;
    //Hanging hang;
    FieldCentricFTCLib drive;
    //Failsafe failsafe;
    private GamepadEx gamepad;
    private FieldCentricFTCLib fieldCentricDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new HWMap(telemetry, hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        cycle = new Cycle(hardware, gamepad, telemetry, hardwareMap);
        state = RobotFSM.start;
        //drone = new DroneLaunch(hardware);
        // hang = new Hanging(hardware);
        drive = new FieldCentricFTCLib(telemetry, hardwareMap);
        //   failsafe = new Failsafe(hardware);
        hardware.getOutakeServoLeft().setPosition(0);
        hardware.getOutakeServoRight().setPosition(0);
        telemetry.addData("-", "Init Done");
        telemetry.update();

        try {
            fieldCentricDrive = new FieldCentricFTCLib(telemetry, hardwareMap);
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
             /*      if (gamepad1.y) {
                       state = RobotFSM.hangingFSM;
                       telemetry.addData("y pressed", 1);
                   }
                   if (gamepad1.a) {
                       state = RobotFSM.droneFSM;
                       telemetry.addData("a pressed", 1);
                   }
                   if (gamepad1.b) {
                       state = RobotFSM.failsafe;
                       telemetry.addData("b pressed", 1);
                   }

              */
                    break;
                case cycleFSM:
                    telemetry.addData("in cycle state", 1);
                    cycle.loop();
                    state = RobotFSM.start;
                    break;
        /*       case hangingFSM:
                   telemetry.addData("in hanging state", 1);
                   hang.loop();
                   state = RobotFSM.start;
                   break;
               case droneFSM:
                   telemetry.addData("in drone state", 1);
                   drone.loop();
                   state = RobotFSM.start;
                   break;
               case failsafe:
                   telemetry.addData("in failsafe state", 1);
                   state = RobotFSM.start;
                   break;

         */
            }
            telemetry.addData("end of Main loop", 1);
            telemetry.update();
            //drivetrain code here
            gamepad.readButtons();

            fieldCentricDrive.drive(gamepad);

    }
            }
        }

