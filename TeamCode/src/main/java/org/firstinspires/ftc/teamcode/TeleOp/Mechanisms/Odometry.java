package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class Odometry {

        private Servo OdoRetractionLeft;
        private Servo OdoRetractionRight;
        private Servo OdoRetractionMiddle;

        public static double leftRetract = 0.05;
        public static double rightRetract = 0;
        public static double middleRetract = 0;
        public static double leftExtend = 0.4;
        public static double rightExtend = 0;
        public static double middleExtend = 0;


        public Odometry(HWMap hwMap) {
            OdoRetractionLeft = hwMap.getOdoRetractionLeft();
            OdoRetractionRight = hwMap.getOdoRetractionRight();
            OdoRetractionMiddle = hwMap.getOdoRetractionMiddle();
        }

        public void retractOdo(){
            OdoRetractionLeft.setPosition(leftRetract);
            OdoRetractionRight.setPosition(rightRetract);
            OdoRetractionMiddle.setPosition(middleRetract);
        }

        public void extendOdo() {
            OdoRetractionLeft.setPosition(leftExtend);
            OdoRetractionRight.setPosition(rightExtend);
            OdoRetractionMiddle.setPosition(middleExtend);
        }
}