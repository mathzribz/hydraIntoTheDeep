
package org.firstinspires.ftc.teamcode.Main;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@TeleOp
public final class AutoSpecimen6 extends LinearOpMode {
    VisionPortal visionPortal;
    AprilTagProcessor tagProcessor;
    WebcamName controlHubCam;


    Pose2d beginPose = null; // Começa indefinida

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        controlHubCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Inicializa o AprilTagProcessor
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setLensIntrinsics(935.41, 935.41, 600.505, 412.42)
                .build();

        // Inicializa a VisionPortal com a câmera do Control Hub
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(controlHubCam) // Ajuste o nome da câmera se necessário
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        telemetry.addLine("Aguardando detecção da AprilTag...");
        telemetry.update();

        // Espera até que uma AprilTag seja detectada
        while (!isStopRequested() && beginPose == null) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();
            for (AprilTagDetection tag : detections) {
                if (tag.id == 11) {  // Filtra a AprilTag desejada


                    double myTagPoseX = -tag.ftcPose.y + 81.5 ;
                    double myTagPoseY = -tag.ftcPose.x - 76.1 ;

                    // Cálculo da posição do robô sem o heading
                    double x = myTagPoseX;
                    double y = myTagPoseY;

                    beginPose = new Pose2d(x, y, 0); // Define a posição inicial sem heading

                    break;
                }
            }

            telemetry.addData("AprilTag Detectada?", beginPose != null);
            telemetry.addData("beginPose", beginPose);
            telemetry.update();
        }

        // Desliga a câmera para economizar processamento
        visionPortal.close();

        // Se nenhuma AprilTag foi detectada, não faz nada
        if (beginPose == null) {
            telemetry.addLine("Nenhuma AprilTag detectada.");
            telemetry.update();
            return;
        }


        // Inicializa o drive com a posição corrigida
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Pose2d finalPose = new Pose2d(6, -35, Math.toRadians(90));

        waitForStart();

        // Executa a trajetória apenas se a AprilTag foi detectada
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(8, -35, 1.6), 0)
                        .waitSeconds(0.3)

                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(38, -60), -0.3)

                        .strafeToLinearHeading(new Vector2d(30, -35), 0.7)
                        .turn(-1.7)
                        .strafeToLinearHeading(new Vector2d(40, -35), 0.7)
                        .turn(-1.7)
                        .strafeToLinearHeading(new Vector2d(50, -35), 0.7)
                        .turn(-2.25)

                        .strafeTo(new Vector2d(50, -58))


// Movimentos repetitivos (subindo e descendo)

                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(11, -35))
                        .waitSeconds(0.3)

                        .strafeTo(new Vector2d(38, -60))
                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(11, -35))
                        .waitSeconds(0.3)

                        .strafeTo(new Vector2d(38, -60))
                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(11, -35))
                        .waitSeconds(0.3)

                        .strafeTo(new Vector2d(38, -60))
                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(11, -35))
                        .waitSeconds(0.3)

                        .strafeTo(new Vector2d(38, -60))
                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(11, -35))

// Estacionamento final com spline otimizada

                        .strafeToConstantHeading(new Vector2d(42, -62) )



                        .build());

    }
}