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
    } //would turn this into a constructor + pass in hardware object instead of instantiating here

    public void loop() { //needs an exit
        switch (state) {
            case start: //would add conditionals here
                state = hangingFSM.one;
                break;
            case one: //state transition should go back to start
                state = hangingFSM.two;
                break;
            case two:
                state = hangingFSM.start;
                break;
            default: //would remove this
                state = hangingFSM.start;
        }

    }
}
