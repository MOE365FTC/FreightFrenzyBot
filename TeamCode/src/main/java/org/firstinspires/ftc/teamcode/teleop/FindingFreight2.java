package org.firstinspires.ftc.teamcode.teleop;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.enums.FreightDirection;
import org.firstinspires.ftc.teamcode.enums.TSEPos;
import org.firstinspires.ftc.teamcode.enums.TurnDirection;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class FindingFreight2 extends OpenCvPipeline {
    Mat mat = new Mat();
    Mat hsv = new Mat();
    Mat mask = new Mat();
    private double xpos = 0;
    double area = 0;
    Mat inverted = new Mat();
    double maxarea = 0;
    private FreightDirection position = FreightDirection.RIGHT;
    int counter = 0;
    double height = 0, width = 0, channels = 0;
    Scalar low_range = new Scalar(10, 150, 150);
    boolean checkifstop = false;
    Scalar upper_range = new Scalar(30, 255, 255);
    Telemetry telemetry;
    // init telemetry

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        height = input.height();
        width = input.width();

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        mask = mat.clone();
        Core.inRange(hsv, low_range, upper_range, mask);
        int whitePixels = 0;
        int blackPixels = 0;

        Mat binary = new Mat(mask.rows(), mask.cols(), mask.type(), new Scalar(0));
        Imgproc.threshold(mask, binary, 100, 255, Imgproc.THRESH_BINARY_INV);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchey = new Mat();
        Imgproc.findContours(binary, contours, hierarchey, Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE);
        counter = 0;
        xpos = 0;
        for (int i = 0; i < contours.size(); i++) {
            double cont_area = Imgproc.contourArea(contours.get(i));
            // value up for change - getting cont_area
            if (cont_area > 200.0 && cont_area < 1000.0) {
                counter++;
                Rect rect = Imgproc.boundingRect(contours.get(i));
                xpos = rect.x;
                area = cont_area;
            }
        }

//        if (xpos == 0.0 || xpos > 120) {
//            position = 1;
//        } else if (xpos > 50 && xpos < 110) {
//            position = 2;
//        } else {
//            position = 3;
//        }
        if (xpos > 47) {
            position = FreightDirection.RIGHT;
        } else {
            position = FreightDirection.STOP;
        }
        return input;
    }

    public FreightDirection getPosition(){
        return position;
    }

    public double getXPos(){
        return xpos;
    }
}