package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.HWMap;


public class FieldCentricFTCLib extends HWMap{
    private HWMap hwMap;
    private DcMotorEx leftFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx rightBackMotor;
    public FieldCentricFTCLib(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);
        hwMap = new HWMap(telemetry, hardwareMap);
        leftBackMotor = hwMap.getLeftBackMotor();
        leftFrontMotor = hwMap.getLeftFrontMotor();
        rightBackMotor = hwMap.getRightBackMotor();
        rightFrontMotor = hwMap.getRightFrontMotor();
    }
    //Read FTC LIB
}
