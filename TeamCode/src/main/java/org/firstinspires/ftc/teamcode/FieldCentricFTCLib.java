package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class FieldCentricFTCLib extends HWMap {
    private HWMap hwMap;
    private Motor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    private MecanumDrive mecanumDrive;

    public FieldCentricFTCLib(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);
        hwMap = new HWMap(telemetry, hardwareMap);
        leftBackMotor = hwMap.getLeftBackMotor();
        leftFrontMotor = hwMap.getLeftFrontMotor();
        rightBackMotor = hwMap.getRightBackMotor();
        rightFrontMotor = hwMap.getRightFrontMotor();
        mecanumDrive = new MecanumDrive(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
    }

    public void drive(double strafe, double forward, double turn, double heading) {
        mecanumDrive.driveFieldCentric(strafe, forward, turn, heading);
    }
}
