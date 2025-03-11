package org.firstinspires.ftc.teamcode.Main;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
public final class AutoSpecimen extends LinearOpMode {
    VisionPortal visionPortal;
    AprilTagProcessor tagProcessor;
    WebcamName controlHubCam;

    boolean er;


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


                    double x = myTagPoseX;
                    double y = myTagPoseY;

                    // Define a posição inicial do robô
                    beginPose = new Pose2d(15, -64, 0);

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

        subsystems.Kit kit = new subsystems().new Kit(hardwareMap);
        subsystems.Ang ang = new subsystems().new Ang(hardwareMap);
        subsystems.Antebraco arm = new subsystems().new Antebraco(hardwareMap);
        subsystems.Pulso pulso = new subsystems().new Pulso(hardwareMap);
        subsystems.Claw claw = new subsystems().new Claw(hardwareMap);



        claw.new ClawClose(); // Fecha a garra
        arm.SetPosition(0); // Posição inicial do braço
        pulso.SetPosition(2); // Posição inicial do pulso
        ang.SetPosition(0); // Posição inicial do ângulo

        waitForStart();
        // Define as trajetórias
        TrajectoryActionBuilder traj1, traj2, traj3, traj4, traj5;


        traj1 = drive.actionBuilder(beginPose)
                . setReversed(true)
                .splineTo(new Vector2d(8, -47), Math.toRadians(90))

        ;

        traj2 = traj1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(8, -50))
        ;
        traj3 = traj2.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(37, -30), Math.toRadians(90))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(37, -9), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(45, -9), Math.toRadians(90))


                .strafeTo(new Vector2d(45, -58)) // Mantém X = 45 e move apenas o Y
                .splineToConstantHeading(new Vector2d(56, -10), Math.toRadians(-30))
                .strafeTo(new Vector2d(60, -58)) // Mantém X = 45 e move apenas o Y
                .splineToConstantHeading(new Vector2d(65, -10), Math.toRadians(-30))
                .strafeTo(new Vector2d(65, -58)) // Mantém X = 45 e move apenas o
        ;




        Actions.runBlocking(
                new SequentialAction(
                        traj1.build(),
                        arm.ArmUp(),
                        ang.AngUp(),
                        traj2.build()


                ));





        if (isStopRequested()) {
            return;
        }


        Actions.runBlocking(
                new ParallelAction(
                        ang.UpdatePID_Ang(),
                        kit.UpdatePID_Kit(),
                        arm.UpdatePID_Antebraço(),
                        pulso.UpdatePID_Pulso()

                )
         );





}
}