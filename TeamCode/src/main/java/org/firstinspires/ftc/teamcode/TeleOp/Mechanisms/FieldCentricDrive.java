package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;


public class FieldCentricDrive {
    private final MecanumDrive mecanumDrive;

    public FieldCentricDrive(HWMap hwMap) {
        mecanumDrive = hwMap.getMecanumDrive();
    }

    public void drive(double strafe, double forward, double turn, double heading) {
        mecanumDrive.driveFieldCentric(strafe, forward, turn, heading);
    }


}