package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class testesMotor extends LinearOpMode {
    private DcMotor KR, AL, KITL, KITR;


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        while (opModeIsActive()) {
            KR = hardwareMap.get(DcMotor.class, "KR"); // porta 0
            AL = hardwareMap.get(DcMotor.class, "AL"); // porta 1



            KR.setDirection(DcMotor.Direction.FORWARD);


            KR.setPower(gamepad2.right_stick_y);



            telemetry.addData("KR", KR.getPower());

            telemetry.update();

        }
    }
}