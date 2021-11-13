package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.TeamMarkerTracker;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OutputtingTeamMarkerPos extends OpMode {
    WebcamName webcamName;
    OpenCvCamera camera;
    TeamMarkerTracker tracker;
    int position = 0;

    @Override
    public void init() {
        webcamName = hardwareMap.get(WebcamName.class, "HighCam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        tracker = new TeamMarkerTracker();
        camera.openCameraDevice();
        camera.startStreaming(160, 90, OpenCvCameraRotation.UPRIGHT);
        camera.setPipeline(tracker);

    }

    @Override
    public void loop() {
//        if (tracker.xcoordinate < tracker.imagex / 3) {
////            position = 1;
////        } else if (tracker.xcoordinate- 300 < (tracker.imagex / 3) * 2) {
////            position = 2;
////        } else {
////            position = 3;
////        }
        FtcDashboard.getInstance().startCameraStream(camera, 60);
        telemetry.addData("position", "somethign");
//        telemetry.addData("iamgex", tracker.imagex);
//        telemetry.addData("counterrr", tracker.counterrr);
//        telemetry.addData("xvalue", tracker.xcoordinate);
//        telemetry.addData("max", tracker.maxx);
        telemetry.addData("counter", tracker.coutner);
        telemetry.addData("chalnes", tracker.channels);
        telemetry.addData("area", tracker.area);
        telemetry.addData("xpos", tracker.xpos);
        telemetry.addData("position", tracker.position);
//        telemetry.addData("imagex", tracker.imagex);
//        telemetry.addData("counter", tracker.counter2);
    }
}
