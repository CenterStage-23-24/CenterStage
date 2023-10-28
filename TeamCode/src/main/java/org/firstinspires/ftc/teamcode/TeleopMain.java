package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TeleopMain extends OpMode {

    public enum RobotFSM {
        start,
        cycleFSM,
        hangingFSM,
        droneFSM,
        failsafe
    }

    RobotFSM state = RobotFSM.start;
    HWMap hardware;
    Cycle cycle;
    DroneLaunch drone;
    Hanging hang;
    FieldCentricDrive drive;

    @Override
    public void init() {

        telemetry.addData("-", "Init Start"); //This code works fine
        telemetry.update();

        hardware = new HWMap(telemetry, hardwareMap); //Tested until here --> error occurs here

        /*
        //cycle = new Cycle(hardware.getIntakeMotor(), hardware.getLinearSlidesRight(), hardware.getLinearSlidesLeft(), hardware.getOutakeServoLeft(), hardware.getOutakeServoRight(), gamepad1);
        drone = new DroneLaunch();
        hang = new Hanging();
        drive = new FieldCentricDrive(telemetry, hardwareMap);
        telemetry.addData("-", "Init Done");
        telemetry.update();
         */
    }

    @Override
    public void loop() {
        switch (state) {
            case start:
                if (gamepad1.x) {
                    state = RobotFSM.cycleFSM;
                }
                break;
            case cycleFSM:
                cycle.loop();
                state = RobotFSM.hangingFSM;
                break;
            case hangingFSM:
                hang.loop();
                state = RobotFSM.droneFSM;
                break;
            case droneFSM:
                drone.loop();
                state = RobotFSM.failsafe;
                break;
            case failsafe:
                state = RobotFSM.start;
                break;
            default:
                state = RobotFSM.start;
        }


    }
}
