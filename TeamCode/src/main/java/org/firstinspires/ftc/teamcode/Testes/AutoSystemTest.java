package org.firstinspires.ftc.teamcode.Testes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.subsystems;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp
public final class AutoSystemTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            subsystems.Kit kit = new subsystems().new Kit(hardwareMap);
            subsystems.Ang ang = new subsystems().new Ang(hardwareMap);
            subsystems.Antebraco arm = new subsystems().new Antebraco(hardwareMap);
            subsystems.Pulso pulso = new subsystems().new Pulso(hardwareMap);
            subsystems.Claw claw = new subsystems().new Claw(hardwareMap);

            Actions.runBlocking(
                    new ParallelAction(
                            claw.new ClawClose()


                    )
            );

            waitForStart();

            TrajectoryActionBuilder move = drive.actionBuilder(beginPose)
                    .afterTime(1.5, ang.SetPosition(50));
//                            .afterTime(1.0, claw.new ClawOpen())
//                            .afterTime(2, claw.new ClawClose())
//                            .afterTime(2, ang.SetPosition(10))
//                            .afterTime(2, arm.SetPosition(10))
//                            .afterTime(2, pulso.SetPosition(10))


            if (isStopRequested()) {
                return;
            }


            Actions.runBlocking(
                    new ParallelAction(
                            move.build(),
                            ang.UpdatePID_Ang(),
                            kit.UpdatePID_Kit(),
                            arm.UpdatePID_Antebraço(),
                            pulso.UpdatePID_Pulso()
                    )
            );


        }
    }
}
