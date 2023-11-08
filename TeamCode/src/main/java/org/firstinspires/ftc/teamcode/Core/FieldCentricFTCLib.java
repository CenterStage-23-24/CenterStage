package org.firstinspires.ftc.teamcode.Core;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.HWMapFTCLib;


public class FieldCentricFTCLib {
    private HWMap hwMap;
    private Motor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    //private DcMotorEx leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    private MecanumDrive mecanumDrive;

    public FieldCentricFTCLib(Telemetry telemetry, HardwareMap hardwareMap, HWMap hwMap) {
        this.hwMap = hwMap;
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