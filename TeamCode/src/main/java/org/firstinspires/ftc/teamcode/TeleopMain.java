package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TeleopMain extends OpMode {
    public TeleopMain () {

    }
    public enum RobotFSM {
        start,
        cycleFSM,
        hangingFSM,
        droneFSM,
        failsafe
    }

    RobotFSM state = RobotFSM.start;
    Telemetry telemetry;
    HWMap hardware = new HWMap(telemetry, hardwareMap);
    Cycle cycle;
    DroneLaunch drone;
    Hanging hang;
    FieldCentricDrive drive;

    @Override
    public void init() {
        telemetry.addLine("Init");
        telemetry.update();
        cycle = new Cycle(hardware.getIntakeMotor(), hardware.getLinearSlidesRight(), hardware.getLinearSlidesLeft(), hardware.getOutakeServoLeft(), hardware.getOutakeServoRight(), gamepad1);
        DroneLaunch drone = new DroneLaunch();
        Hanging hang = new Hanging();
        FieldCentricDrive drive = new FieldCentricDrive(telemetry, hardwareMap);
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
