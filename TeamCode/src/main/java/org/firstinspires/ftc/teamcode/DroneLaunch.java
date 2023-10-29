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
    } //would turn this into a constructor + pass in hardware object instead of instantiating here

    public void loop() { //needs an exit
        switch (state) {
            case start: //would add conditionals here
                state = DroneLaunch.droneFSM.one;

                break;
            case one: //state transition should go back to start
                state = DroneLaunch.droneFSM.two;
                break;
            case two:
                state = DroneLaunch.droneFSM.start;
                break;
            default: //would remove this
                state = droneFSM.start;
        }

    }
}
