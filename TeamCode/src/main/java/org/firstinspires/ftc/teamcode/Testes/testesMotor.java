package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class testesMotor extends LinearOpMode {
    private DcMotor AR, AL, KITL, KITR;


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        while (opModeIsActive()) {
            AR = hardwareMap.get(DcMotor.class, "Arm"); // porta 0
            AL = hardwareMap.get(DcMotor.class, "AL"); // porta 1



            AR.setDirection(DcMotor.Direction.FORWARD);


            AR.setPower(gamepad2.right_stick_y);



            telemetry.addData("AR", AR.getPower());

            telemetry.update();

        }
    }
}