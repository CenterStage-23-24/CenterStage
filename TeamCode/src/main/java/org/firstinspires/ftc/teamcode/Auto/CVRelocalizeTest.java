package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Autonomous
public class CVRelocalizeTest extends LinearOpMode {
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private ArrayList<String> tagIDs = new ArrayList<>();

    @Override
    public void runOpMode(){
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

        waitForStart();

        double lastValue = 0.0;
        while(opModeIsActive()){
            telemetry.addData("last range seen", lastValue);
            if (lastValue < 4.0) {
                telemetry.addLine("####################");
                telemetry.addLine("WENT LESS THAN 4.0 INCHES FROM APRIL TAG!!!");
                telemetry.addLine("####################");
            }
            // 3.44, 3.74, 3.734, 3.642

            if(tagProcessor.getDetections().size() != 0){
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                lastValue = tag.ftcPose.range;
                telemetry.addData("range", tag.ftcPose.range);
                telemetry.addData("bearing", tag.ftcPose.bearing);
                telemetry.addData("elevation", tag.ftcPose.elevation);
                telemetry.addData("tag ID", tag.id);
            }


            telemetry.addData("tags", tagProcessor.getDetections());
            telemetry.update();
        }
    }
}
