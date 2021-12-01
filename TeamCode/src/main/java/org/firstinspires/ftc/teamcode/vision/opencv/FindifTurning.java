package org.firstinspires.ftc.teamcode.vision.opencv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class FindifTurning extends OpenCvPipeline {
    Mat mat = new Mat();
    Mat hsv = new Mat();
    Mat mask = new Mat();
    int largestx = 0, width = 0, height, maxheightdivided;
    int coutner = 0;
    Scalar low_range = new Scalar(0, 0, 0);
    Scalar upper_range = new Scalar(180, 255, 40);
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        width = input.width();
        height = input.height();
        maxheightdivided = (int) (((height / 4) * 5));
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        mask = mat.clone();
        Rect rectCrop = new Rect(0, 0, maxheightdivided, height);
        mask = new Mat(mask, rectCrop);
        // crop mask
        Core.inRange(hsv, low_range, upper_range, mask);
        Mat binary = new Mat(mask.rows(), mask.cols(), mask.type(), new Scalar(0));
        Imgproc.threshold(mask, binary, 100, 255, Imgproc.THRESH_BINARY_INV);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchey = new Mat();
        Imgproc.findContours(binary, contours, hierarchey, Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE);
        coutner = 0;
        largestx = 0;
        for (int i=0; i < contours.size(); i++) {
            double cont_area = Imgproc.contourArea(contours.get(i));
            Rect rect = Imgproc.boundingRect(contours.get(i));
            if (cont_area > 20000.0 && rect.x == 0.0) {
                coutner++;
                largestx = Math.max(largestx, rect.width);
            }
        }
        return input;
    }
}