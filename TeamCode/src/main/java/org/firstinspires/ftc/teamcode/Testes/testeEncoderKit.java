package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class testeEncoderKit extends LinearOpMode {
    private DcMotor KL, KR;
    private final double ticksMax = 6430;

    public void manualControl() {

        KR = hardwareMap.get(DcMotor.class, "KR");
        telemetry.addData("Hardware", "Inicializado");

        // Resetar e configurar encoders
        KR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

     // KR sem restrição de encoder

        // Configuração de direção

        KR.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        manualControl();

        while (opModeIsActive()) {
            // Verificar se o limite foi atingido
            if (KL.getCurrentPosition() >= ticksMax) {
                if (-gamepad1.left_stick_y > 0) { // Tentativa de subir

                    KR.setPower(0);
                    telemetry.addData("TicksMax atingido", "Movimento para cima bloqueado");
                } else if (gamepad1.left_stick_y > 0) { // Movimento para baixo permitido

                    KR.setPower(-gamepad1.left_stick_y);
                    telemetry.addData("Movimento permitido", "Descendo");
                }
            } else {
                // Movimentos permitidos livremente quando limite não foi atingido

                KR.setPower(-gamepad1.left_stick_y);
                telemetry.addData("Movimento livre", "Limite não atingido");
            }

            // Telemetria para monitoramento
            telemetry.addData("KL ticks", KR.getCurrentPosition());
            telemetry.addData("KR power", KR.getPower());
            telemetry.addData("ticksMax", ticksMax);
            telemetry.update();
        }
    }
}
