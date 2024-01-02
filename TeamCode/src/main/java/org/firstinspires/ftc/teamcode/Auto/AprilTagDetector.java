package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TeleOp.Mechanisms.HWMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagDetector {
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private Motor leftFrontMotor;
    private Motor rightFrontMotor;
    private Motor rightBackMotor;
    private Motor leftBackMotor;
    private HWMap hwMap;
    private double yaw;
    private double bearing;
    private double range;
    private AprilTagDetection prev_tag;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        hwMap = new HWMap(hardwareMap);

        leftFrontMotor = hwMap.getLeftFrontMotor();
        rightFrontMotor = hwMap.getRightFrontMotor();
        rightBackMotor = hwMap.getRightBackMotor();
        leftBackMotor = hwMap.getLeftBackMotor();

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    public void detect(){
        if(tagProcessor.getDetections().size() > 0){
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            yaw = tag.ftcPose.yaw;
            bearing = tag.ftcPose.bearing;
            range = tag.ftcPose.range;
        }
    }

    public double getYaw(){
        return yaw;
    }

    public double getBearing(){
        return bearing;
    }
    public double getDetections(){return tagProcessor.getDetections().size();}
    public double getRange(){return range;}
}