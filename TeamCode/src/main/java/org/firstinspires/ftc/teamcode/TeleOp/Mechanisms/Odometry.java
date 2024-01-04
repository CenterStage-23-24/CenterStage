package org.firstinspires.ftc.teamcode.TeleOp.Mechanisms;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Odometry {

        private Servo OdoRetractionLeft;
        private Servo OdoRetractionRight;
        private Servo OdoRetractionMiddle;

        private double leftRetract = 0.75;
        private double rightRetract = 0;
        private double middleRetract = 0;
        private double leftExtend = 0;
        private double rightExtend = 0;
        private double middleExtend = 0;

        public Odometry(HWMap hwMap) {
            OdoRetractionLeft = hwMap.getOdoRetractionLeft();
            //OdoRetractionRight = hwMap.getOdoRetractionRight();
            OdoRetractionMiddle = hwMap.getOdoRetractionMiddle();
        }

        public void retractOdo(){
            OdoRetractionLeft.setPosition(leftRetract);
            //OdoRetractionRight.setPosition(rightRetract);
            OdoRetractionMiddle.setPosition(middleRetract);
        }

        public void extendOdo() {
            OdoRetractionLeft.setPosition(leftExtend);
            //OdoRetractionRight.setPosition(rightExtend);
            OdoRetractionMiddle.setPosition(middleExtend);
        }
}