package org.firstinspires.ftc.teamcode.OpenCV;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        return input;
    }
}
