package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

public class Detector {
    OpenCvWebcam camera;
    PropPipeline propPipeline;
    public Detector(HardwareMap hardwareMap, Telemetry telemetry){
        propPipeline = new PropPipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //camera.setMillisecondsPermissionTimeout(5000);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("-", "Camera Streaming!");
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("DETECTOR ERROR: ", errorCode);
            }
        });
        telemetry.addData("-", "Detector construction over!");
    }

    public void detect(){
        camera.setPipeline(propPipeline);
    }

    public double getX(){
        return propPipeline.getX();
    }
    public double getY(){
        return propPipeline.getY();
    }


    public int getFindContourNum(){
        return propPipeline.getFindContourNum();
    }
    public int getFilterContourNum(){
        return propPipeline.getFilterContourNum();
    }
    public String getPosition(){
        return propPipeline.getPosition();
    }
    public ArrayList<Double> getContourAreas(){
        return propPipeline.getContourAreas();
    }
    public void switchPipeline(OpenCvPipeline pipeline){camera.setPipeline(pipeline);}
}