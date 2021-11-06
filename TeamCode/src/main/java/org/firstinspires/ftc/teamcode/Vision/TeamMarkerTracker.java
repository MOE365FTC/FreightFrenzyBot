package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
@Config
public class TeamMarkerTracker extends OpenCvPipeline {
    public static int lowHue = 105;
    public static int lowSat = 15;
    public static int lowVal = 65;
    public static int highHue = 130;
    public static int highSat = 50;
    public static int highVal = 35;
    Mat mat = new Mat();
    Mat blue = new Mat();
    Mat yellow = new Mat();
    double imagex, iamgey;
    double xcoordinate = 0;
    double maxx = 0;
    int counterrr = 0;
    int counter2 = 0;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HSV);

        Size s = mat.size();
        imagex = s.width;
        Scalar lowblue = new Scalar(lowHue, lowSat, lowVal);
        Scalar highblue = new Scalar(highHue, highSat, highVal);
//
        Core.inRange(mat, lowblue, highblue, blue);
        Mat gray = new Mat(blue.rows(), blue.cols(), blue.type());
        Imgproc.cvtColor(mat, gray, Imgproc.COLOR_BGR2GRAY);
        Mat binary = new Mat(blue.rows(), blue.cols(), blue.type(), new Scalar(0));
        Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY_INV);
//        //Finding Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchey = new Mat();
        Imgproc.findContours(binary, contours, hierarchey, Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE);
        int counter = 0;
        counter2 = 0;
        double maxcounter = 0;
        maxx = 0;
        for (int i = 0; i < contours.size(); i++) {
            double cont_area = Imgproc.contourArea(contours.get(i));
            counter2++;
            if (cont_area > maxcounter) {
                maxcounter = cont_area;
                counter = i;
                maxx = cont_area;
            }
        }
        counterrr = 0;
        xcoordinate = 0;
        for (int i = 0; i < contours.size(); i++) {
            if (i == counter) {
                counterrr += 1;
                Rect boundingrect = Imgproc.boundingRect(contours.get(i));
                xcoordinate = boundingrect.x + boundingrect.width;
            }
        }
//        telemetry.addData("letsee", imagex);
        return input;
    }
}
