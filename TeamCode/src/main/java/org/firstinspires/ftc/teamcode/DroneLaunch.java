package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLaunch {
    public enum droneFSM {
        start,
        one,
        two
    }
    // this is a test right now
    droneFSM state = droneFSM.start;
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public void init() {
        HWMap hardware = new HWMap(telemetry, hardwareMap);
    }

    public void loop() {
        switch (state) {
            case start:
                state = DroneLaunch.droneFSM.one;

                break;
            case one:
                state = DroneLaunch.droneFSM.two;
                break;
            case two:
                state = DroneLaunch.droneFSM.start;
                break;
            default:
                state = droneFSM.start;
        }

    }
}
