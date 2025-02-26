package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.mechanisms.subsystems;

public final class systemTest extends LinearOpMode {
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



            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .afterTime(1.5, kit.SetPosition(10))
//                            .afterTime(1.0, claw.new ClawOpen())
//                            .afterTime(2, claw.new ClawClose())
//                            .afterTime(2, ang.SetPosition(10))
//                            .afterTime(2, arm.SetPosition(10))
//                            .afterTime(2, pulso.SetPosition(10))
                            .turn(2)
                            .build());
            ;
        }
    }
}
