package org.firstinspires.ftc.teamcode.Testes;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class testePIDFArmsDual extends OpMode {

    private PIDFController controller;

    // Constantes de PIDF e alvo inicial
    public static double p = 0.01, i = 0.0, d = 0.000;
    public static double f = 0.1; // Feedforward inicial
    public static int target; // Alvo inicial em ticks

    // Fator de conversão para graus
    public final double ticks_in_degrees = 360.0 / 28; // Para Through Bore Encoder da REV

    // Motores e Encoder Externo
    private DcMotorEx AR, AL; // Motores do braço

    private boolean isManualControl = true;

    @Override
    public void init() {
        // Inicialização do controlador PIDF
        controller = new PIDFController(p, i, d, f);

        // Configuração do telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Inicialização dos motores
        AR = hardwareMap.get(DcMotorEx.class, "AR"); // Braço Direito
        AL = hardwareMap.get(DcMotorEx.class, "AL"); // Braço Esquerdo


        AR.setDirection(DcMotorEx.Direction.REVERSE);
        AL.setDirection(DcMotorEx.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if (gamepad1.touchpad) {
            isManualControl = !isManualControl;
            // Pequeno delay para evitar múltiplos registros do toque
        }
        if (isManualControl) {

        } else {




        }
        // Atualizar valores de PIDF em tempo real
        controller.setPIDF(p, i, d, f);

        // Obter a posição do encoder externo
        int posPivot = AR.getCurrentPosition(); // Agora está correto!

        // Calcular PID com base na posição do encoder
        double pid = controller.calculate(posPivot, target);

        // Calcular Feedforward
        double ff = Math.cos(Math.toRadians(target * ticks_in_degrees)) * f;

        // Calcular potência final
        double output = pid + ff;
        output = Math.max(-1.0, Math.min(1.0, output));

        // Ajuste de direção para evitar que os motores se batam
        AR.setPower(output);
        AL.setPower(-output);

        // Exibir informações no telemetry
        telemetry.addData("Posição Encoder (ticks):", posPivot);
        telemetry.addData("Alvo (ticks):", target);
        telemetry.addData("Potência Final:", output);
        telemetry.addData("PID:", pid);
        telemetry.addData("Feedforward:", ff);
        telemetry.addData("power AR", AR.getPower());
        telemetry.addData("power AL", AL.getPower());
        telemetry.update();
    }
}

