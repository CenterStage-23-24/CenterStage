package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TeleOp.HWMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class FieldCentricFTCLib {
    private HWMap hwMap;
    private Motor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    private MecanumDrive mecanumDrive;

    public FieldCentricFTCLib(HWMap hwMap) {
        this.hwMap = hwMap;
        leftBackMotor = hwMap.getLeftBackMotor();
        leftFrontMotor = hwMap.getLeftFrontMotor();
        rightBackMotor = hwMap.getRightBackMotor();
        rightFrontMotor = hwMap.getRightFrontMotor();
        mecanumDrive = new MecanumDrive(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
        mecanumDrive.setRightSideInverted(false);

    }

    public void drive(double strafe, double forward, double turn, double heading) {
        mecanumDrive.driveFieldCentric(strafe, forward, turn, heading);
    }


}