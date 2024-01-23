package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class CVRelocalizer {
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    public CVRelocalizer(HardwareMap hwMap) {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    public AprilTagPoseFtc getFtcPose(int id) {
        AprilTagDetection tag = getTagFromId(id);

        if (tag == null)
            return null;

        return tag.ftcPose;
    }

    public double getYaw(int id){
        AprilTagDetection tag = getTagFromId(id);

        if (tag == null)
            return 0.0;

        return tag.ftcPose.yaw;
    }

    public double getX(int id){
        AprilTagDetection tag = getTagFromId(id);

        if (tag == null)
            return 0.0;

        return tag.ftcPose.x;
    }

    private AprilTagDetection getTagFromId(int id) {
        for(AprilTagDetection tag : tagProcessor.getDetections()){
            if(tag.id == id){
                return tag;
            }
        }

        return null;
    }
}
