package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp
public class testeServo extends LinearOpMode {
    public Servo servoG,  servoA, servoP;


    public void initservoA () {

        servoG = hardwareMap.get(Servo.class, "servoG");
        servoG.setDirection(Servo.Direction.FORWARD);

        servoP = hardwareMap.get(Servo.class, "servoP");

    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {
            initservoA();
            if (gamepad1.left_bumper) {
                servoG.setPosition(0.6);

            }
            if (gamepad1.right_bumper) {
                servoG.setPosition(0);

            }



            if (gamepad1.a) {
                servoP.setPosition(0);

            }
            if (gamepad1.b) {
                servoP.setPosition(0.8);

            }




            telemetry.addData("pos servoG", servoG.getPosition());
            telemetry.addData("pos servoP", servoP.getPosition());
            telemetry.update();

        }
    }}
