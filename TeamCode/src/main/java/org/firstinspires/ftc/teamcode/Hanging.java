package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hanging {
    public enum hangingFSM {
        start,
        one,
        two
    }
    // this is a test right now
    hangingFSM state = Hanging.hangingFSM.start;
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public void init() {
        HWMap hardware = new HWMap(telemetry, hardwareMap);
    }

    public  void loop() {
        switch (state) {
            case start:
                state = hangingFSM.one;
                break;
            case one:
                state = hangingFSM.two;
                break;
            case two:
                state = hangingFSM.start;
                break;
            default:
                state = hangingFSM.start;
        }

    }
}
