package org.firstinspires.ftc.teamcode.Testes;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;

@TeleOp

public class ApriltagFirst extends LinearOpMode {
    Servo garra;


    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName controlHubCam;
        controlHubCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setLensIntrinsics(935.41, 935.41, 600.505, 412.42)
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())

                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera( controlHubCam)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)


                .build();



        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {


            List<AprilTagDetection> currentDetections = tagProcessor.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            for (AprilTagDetection detection : currentDetections) {
                if (tagProcessor.getDetections().size() > 0) {
                    AprilTagDetection tag = tagProcessor.getDetections().get(0);
                    double myTagPoseX = -tag.ftcPose.y + 81.5 ;
                    double myTagPoseY = -tag.ftcPose.x - 76.1 ;


                    if (detection.ftcPose != null) {
                        telemetry.addData("x", myTagPoseX);
                        telemetry.addData("y", myTagPoseY);


                    }

                }
                else {
                    telemetry.addLine("ftcpose dont found");
                }


//                if (detection.id == 11) {
//                    garra.setPosition(1);
////                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
////                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
////                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//
//                } else if (detection.id == 13) {
//                    garra.setPosition(0.5);
//
//                }
                telemetry.addLine(String.format("\n==== (ID %d) ", detection.id));

            }
            telemetry.update();
        }
    }
}




