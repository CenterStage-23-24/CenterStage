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

    RobotFSM state;
    HWMap hardware;
    Cycle cycle;
    DroneLaunch drone;
    Hanging hang;
    FieldCentricDrive drive;

    @Override
    public void init() { //works

        hardware = new HWMap(telemetry, hardwareMap);
        state = RobotFSM.start;
        cycle = new Cycle(hardware, gamepad1, telemetry);
        drone = new DroneLaunch(hardware);
        hang = new Hanging(hardware);
        drive = new FieldCentricDrive(telemetry, hardwareMap);
        telemetry.addData("-", "Init Done");
        telemetry.update();
    }

        @Override
        public void loop () {
            switch (state) { //exit state?
                case start:
                    telemetry.addLine("in start");
                    telemetry.update();
                    if (gamepad1.x) {
                        telemetry.addLine("X pressed");
                        telemetry.update();
                        state = RobotFSM.cycleFSM;
                    }
                    if (gamepad1.y) {
                        state = RobotFSM.hangingFSM;
                    }
                    if (gamepad1.a) {
                        state = RobotFSM.droneFSM;
                    }
                    if (gamepad1.b) {
                        state = RobotFSM.failsafe;
                    }
                    break;
                case cycleFSM:
                    cycle.loop();
                    state = RobotFSM.start;
                    break;
                case hangingFSM:
                    hang.loop();
                    state = RobotFSM.start;
                    break;
                case droneFSM:
                    drone.loop();
                    state = RobotFSM.start;
                    break;
                case failsafe:
                    state = RobotFSM.start;
                    break;
            }
            telemetry.addLine("end of Main loop");
            telemetry.update();
            //drivetrain code here
        }
}
