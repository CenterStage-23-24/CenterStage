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
    public void init() { //works

        hardware = new HWMap(telemetry, hardwareMap);

        cycle = new Cycle(hardware.getIntakeMotor(), hardware.getLinearSlidesRight(), hardware.getLinearSlidesLeft(), hardware.getOutakeServoLeft(), hardware.getOutakeServoRight(), gamepad1);
        drone = new DroneLaunch();
        hang = new Hanging();
        drive = new FieldCentricDrive(telemetry, hardwareMap);
        telemetry.addData("-", "Init Done");
        telemetry.update();
    }

    @Override
    public void loop() {
        switch (state) { //exit state?
            case start: //needs more conditionals for other states
                if (gamepad1.x) {
                    state = RobotFSM.cycleFSM;
                }
                break;
            case cycleFSM: //state transition should go back to start
                cycle.loop();
                state = RobotFSM.hangingFSM;
                break;
            case hangingFSM: //state transition should go back to start
                hang.loop();
                state = RobotFSM.droneFSM;
                break;
            case droneFSM: //state transition should go back to start
                drone.loop();
                state = RobotFSM.failsafe;
                break;
            case failsafe:
                state = RobotFSM.start;
                break;
            default: //would remove this
                state = RobotFSM.start;
        }

        //drivetrain code here
    }
}
