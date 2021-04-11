package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingDetector {
    OpMode opMode;
    OpenCvCamera camera;

    CustomPipeline pipeline;

    private final Point BLUE_TL    = new Point(90,110);
    private final Point BLUE_BR    = new Point(160, 160);

    Scalar lowHSV = new Scalar(10, 145, 124,0);
    Scalar highHSV = new Scalar(18, 255, 255,0);


//    private final Point RED_TOP_TL     = new Point(0,135);
//    private final Point RED_TOP_BR     = new Point(50, 165);


    private Point TL;
    private Point BR;
    //private Point bottomTL;
    //private Point bottomBR;

    private RGBColor box;

    public RingDetector(OpMode op){

        opMode = op;

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CustomPipeline();
        camera.openCameraDevice();
        camera.setPipeline(pipeline);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        TL = BLUE_TL;
        BR = BLUE_BR;
        //gives points depending on red or blue
        //topTL = (isRed) ? RED_TOP_TL : BLUE_TOP_TL;
        //topBR = (isRed) ? RED_TOP_BR : BLUE_TOP_BR;

    }

    public void stopStreaming(){
        camera.stopStreaming();
    }

    public int getDecision(){
        int boxValue = box.getValueB();
        opMode.telemetry.addData("Value: ", boxValue);
        //opMode.telemetry.addData("Top :" + " R " + box.getRed() + " G " + box.getGreen() + " B " + box.getBlue() + "Y" + box.getYellow() + "Value A"+ box.getValueA()+ "Value B" + box.getValueB() +"Decision: ", getDecision() );
        opMode.telemetry.update();
        boolean topRing = false;
        boolean bottomRing = false;
        if (boxValue < 70) {
            return 4;
        } else if (boxValue < 145){
            return 1;
        } else{
            return 0;
        }
    }

    class CustomPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input){
            Mat processedInput = new Mat();
           // Core.inRange(input, lowHSV, highHSV, processedInput);
            box = getAverageColor(input, TL, BR);
            int thickness = 3;
            //Scalar topColor = new Scalar(255,0,0);
            Scalar Color = new Scalar(255,0,0);
            int position = getDecision();
            if (position == 4){
                Color = new Scalar(0,255,0);
            }
            else if (position == 1){
                Color = new Scalar(0,255,0);
            }

            Imgproc.rectangle(input, TL, BR, Color, thickness);

            //sendTelemetry();

            return input;
        }

        private RGBColor getAverageColor(Mat mat, Point topLeft, Point bottomRight){
            int red = 0;
            int green = 0;
            int blue = 0;
            int total = 0;

            for (int x = (int)topLeft.x; x < bottomRight.x; x++){
                for (int y = (int)topLeft.y; y < bottomRight.y; y++){
                    red += mat.get(y,x)[0];
                    green += mat.get(y,x)[1];
                    blue += mat.get(y,x)[2];
                    total++;
                }
            }

            red /= total;
            green /= total;
            blue /= total;

            return new RGBColor(red, green, blue);
        }

        public void sendTelemetry(){
            opMode.telemetry.addData("Top :" + " R " + box.getRed() + " G " + box.getGreen() + " B " + box.getBlue() + "Y" + box.getYellow() + "Value A"+ box.getValueA()+ "Value B" + box.getValueB() +"Decision: ", getDecision() );
            opMode.telemetry.update();
        }

    }
}
