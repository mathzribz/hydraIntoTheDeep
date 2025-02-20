
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
public final class AutoSpecimen extends LinearOpMode {
    VisionPortal visionPortal;
    AprilTagProcessor tagProcessor;
    WebcamName controlHubCam;


    Pose2d beginPose = null; // Começa indefinida

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance();

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
                if (tag.id == 11) { // Verifica se a AprilTag detectada é a correta
                    beginPose =(new Pose2d(15, -64, 0)); // Define a posição inicial
                    visionPortal.close();
                    break;
                }
            }
            telemetry.addData("AprilTag Detectada?", beginPose != null);
            telemetry.update();
        }

        // Desliga a câmera para economizar processamento


        // Se nenhuma AprilTag foi detectada, não faz nada
        if (beginPose == null) {
            telemetry.addLine("Nenhuma AprilTag detectada. Cancelando...");
            telemetry.update();
            return;
        }

        telemetry.addLine("AprilTag detectada! Iniciando trajetória...");
        telemetry.update();

        // Inicializa o drive com a posição corrigida
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Pose2d finalPose = new Pose2d(6, -35, Math.toRadians(90));

        waitForStart();

        // Executa a trajetória apenas se a AprilTag foi detectada
        Actions.runBlocking(
                drive.actionBuilder(beginPose)

// Primeira movimentação

                        .setReversed(true)
                        .splineTo(new Vector2d(6, -35), Math.toRadians(90))
                        .waitSeconds(0.3)

                        .strafeTo(new Vector2d(33, -35))

                        .strafeTo(new Vector2d(45, -10))
                        .strafeTo(new Vector2d(45, -58)) // Mantém X = 45 e move apenas o Y
                        .splineToConstantHeading(new Vector2d(56, -10), Math.toRadians(-30))
                        .strafeTo(new Vector2d(56, -58)) // Mantém X = 45 e move apenas o Y
                        .splineToConstantHeading(new Vector2d(65, -10), Math.toRadians(-30))
                        .strafeTo(new Vector2d(65, -58)) // Mantém X = 45 e move apenas o

// Movimentos repetitivos (subindo e descendo)

                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(3, -35))
                        .waitSeconds(0.3)

                        .strafeTo(new Vector2d(38, -60))
                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(3, -35))
                        .waitSeconds(0.3)

                        .strafeTo(new Vector2d(38, -60))
                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(3, -35))
                        .waitSeconds(0.3)

                        .strafeTo(new Vector2d(38, -60))
                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(3, -35))
                        .waitSeconds(0.3)

// Estacionamento final com spline otimizada

                        .strafeToConstantHeading(new Vector2d(42, -62) )


                        .build());
    }
}