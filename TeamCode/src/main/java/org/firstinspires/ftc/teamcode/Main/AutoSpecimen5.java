
package org.firstinspires.ftc.teamcode.Main;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.subsystems;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@TeleOp
public final class AutoSpecimen5 extends LinearOpMode {
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
                if (tag.id == 11) {

    // Offset da tag em relação à posição real na arena
                    double tagOffsetX = 81.5;
                    double tagOffsetY = -76.1;

    // Converte as coordenadas da AprilTag para o sistema real da arena
                    double myTagPoseX = -tag.ftcPose.y + tagOffsetX;
                    double myTagPoseY = -tag.ftcPose.x + tagOffsetY;

    // Aplica um fator de escala para compensar distorções de medição


                    double x = myTagPoseX  ;
                    double y = myTagPoseY ;

    // Define a posição inicial do robô
                    beginPose = new Pose2d(x, y, 0);

                    break;
                }
            }

            telemetry.addData("AprilTag Detectada?", beginPose != null);
            telemetry.addData("beginPose", beginPose);
            telemetry.update();
        }

        // Desliga a câmera para economizar processamento


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

        subsystems.Kit kit = new subsystems().new Kit(hardwareMap);
        subsystems.Ang ang = new subsystems().new Ang(hardwareMap);
        subsystems.Antebraco arm = new subsystems().new Antebraco(hardwareMap);
        subsystems.Pulso pulso = new subsystems().new Pulso(hardwareMap);
        subsystems.Claw claw = new subsystems().new Claw(hardwareMap);

        waitForStart();

        TrajectoryActionBuilder move = drive.actionBuilder(beginPose)
            //    .afterTime(0, claw.new ClawClose())
                .setReversed(true)
                .splineTo(new Vector2d(8, -43), Math.toRadians(90))
              //  .afterTime(1, arm.SetPosition(-105))
               // .afterTime(2, ang.SetPosition(378));
//                            .afterTime(1.0, claw.new ClawOpen())
//                            .afterTime(2, claw.new ClawClose())
//                            .afterTime(2, ang.SetPosition(10))
//                            .afterTime(2, arm.SetPosition(10))


                .splineToConstantHeading(new Vector2d(33, -38), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(43, -10), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(50, -58), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46, -10), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(62, -58), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46, -10), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(62, -58), Math.toRadians(90))




// Movimentos repetitivos (subindo e descendo)

                .waitSeconds(0.3)
                .strafeTo(new Vector2d(11, -35))
                .waitSeconds(0.3)

                .strafeTo(new Vector2d(38, -58))
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(11, -35))
                .waitSeconds(0.3)

                .strafeTo(new Vector2d(38, -58))
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(11, -35))
                .waitSeconds(0.3)

                .strafeTo(new Vector2d(38, -58))
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(11, -35))
                .waitSeconds(0.3)

// Estacionamento final com spline otimizada

                .strafeToConstantHeading(new Vector2d(42, -62) );

//

        Actions.runBlocking(
                new SequentialAction(
                   //     claw.new ClawClose(),
                  //      pulso.SetPosition(4),
                         move.build()



                )
        );

        if (isStopRequested()) {
            return;
        }


        Actions.runBlocking(
                new ParallelAction(


                  //      ang.UpdatePID_Ang(),
                  //      kit.UpdatePID_Kit(),
                  //      arm.UpdatePID_Antebraço(),
                  //      pulso.UpdatePID_Pulso()
                )
        );


/*
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

// Estacionamento final com spline otimizada

                        .strafeToConstantHeading(new Vector2d(42, -62) )

*/


    }
}