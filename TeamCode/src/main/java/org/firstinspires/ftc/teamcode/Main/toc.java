package org.firstinspires.ftc.teamcode.Main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class toc extends LinearOpMode {


    private TouchSensor mag, toc;
    @Override
    public void runOpMode() throws InterruptedException {
        toc = hardwareMap.get(TouchSensor.class, "toc"); // porta 0 digial expension
        mag = hardwareMap.get(TouchSensor.class, "mag"); // porta 0 digial expension

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("pressionado ", toc.isPressed());
            telemetry.addData("pressionado dad", mag.isPressed());

            telemetry.update();
        }
    }
}
