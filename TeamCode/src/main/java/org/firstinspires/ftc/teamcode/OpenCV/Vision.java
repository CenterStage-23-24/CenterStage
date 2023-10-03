package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "April tag detection")
public class Vision extends LinearOpMode {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        waitForStart();
        while(opModeIsActive()){
            if(tagProcessor.getDetections().size() > 0){
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addLine(String.format("X: %6.2f Y: %6.2f Z: %6.2f", tag.ftcPose.x, tag.ftcPose.y,tag.ftcPose.z));

                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("yaw", tag.ftcPose.yaw);


            }
            telemetry.update();
        }
    }
}
