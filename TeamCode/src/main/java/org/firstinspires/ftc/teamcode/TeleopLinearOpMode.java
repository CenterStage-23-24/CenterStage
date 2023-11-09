package org.firstinspires.ftc.teamcode;

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
    private Controller gamePad1;
    private Controller gamePad2;
    private FieldCentricFTCLib fieldCentricDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new HWMap(telemetry, hardwareMap);
        gamePad1 = new Controller(gamepad1);
        gamePad2 = new Controller(gamepad2);
        cycle = new Cycle(hardware, gamePad1, telemetry);
        state = RobotFSM.cycleFSM; //CHANGE ONCE TESTING IS DONE
        //drone = new DroneLaunch(hardware);
        // hang = new Hanging(hardware);
        drive = new FieldCentricFTCLib(telemetry, hardwareMap);
        //   failsafe = new Failsafe(hardware);
        hardware.getOutakeServoLeft().setPosition(0);
        hardware.getOutakeServoRight().setPosition(0);
        telemetry.addData("-", "Init Done");
        telemetry.update();

        try {
            gamePad1 = new Controller(gamepad1);
            fieldCentricDrive = new FieldCentricFTCLib(telemetry, hardwareMap);
        } catch (Exception exception) {
            telemetry.addLine("Outside of the while loop:");
            telemetry.addLine(exception.getMessage());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            gamePad1.readButtons();
            gamePad2.readButtons();
            switch (state) { //exit state?
                case start:
                    telemetry.addData("in start", 1);
                    if (gamepad1.x) {
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
            gamePad1.readButtons();


            //FIELD-CENTERIC_______________________________________________________________________________
            double gamepadX;
            double gamepadY;
            double gamepadRot;

            if (Math.abs(gamePad1.gamepadX) > 0.01) {
                gamepadX = gamePad1.gamepadX;
            } //else if (Math.abs(gamePad2.gamepad2X) > 0.01) {
            //gamepadX = controller.gamepad2X; }
        else{
            gamepadX = 0;
        }
        if (Math.abs(gamePad1.gamepadY) > 0.01) {
            gamepadY = gamePad1.gamepadY;
        } // else if (Math.abs(gamePad1.gamepad2Y) > 0.01) {
            //gamepadY = gamePad1.gamepad2Y; }
        else {
            gamepadY = 0;
        }
        if (Math.abs(gamePad1.gamepadRot) > 0.01) {
            gamepadRot = -gamePad1.gamepadRot;
        } //else if (Math.abs(controller.gamepad2Rot) > 0.01) {
            //gamepadRot = -controller.gamepad2Rot; }
        else {
            gamepadRot = 0;
        }

        fieldCentricDrive.drive(gamepadX, gamepadY, gamepadRot, HWMapFTCLib.readFromIMU());

    }
            }
        }

