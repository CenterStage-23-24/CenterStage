package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOp.HWMap;

public class FieldCentricDrive {
    private HWMap hwMap;
    private MecanumDrive mecanumDrive;

    public FieldCentricDrive(Telemetry telemetry, HardwareMap hardwareMap) {
        hwMap = new HWMap(telemetry,hardwareMap);
        mecanumDrive = hwMap.getMecanumDrive();
    }

    public void drive(double strafe, double forward, double turn, double heading) {
        mecanumDrive.driveFieldCentric(strafe, forward, turn, heading);
    }


}