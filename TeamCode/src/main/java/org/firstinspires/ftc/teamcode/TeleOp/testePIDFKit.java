
package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class testePIDFKit extends OpMode {

    private PIDFController controller;

    // Constantes de PIDF e alvo inicial
    public static double p = 0.01, i = 0.0, d = 0;
    public static double f = 0.1; // Feedforward inicial
    public static int target = 0; // Alvo inicial em ticks

    // Fator de conversão (12.8571 graus por tick ou ~0.07778 ticks por grau)
    public final double ticks_in_degrees = 360 / 28.0; // Graus por tick

    // Motores
    private DcMotorEx KL, KR; // KL = Motor com encoder, KR = Motor sem encoder

    @Override
    public void init() {
        // Inicialização do controlador PIDF
        controller = new PIDFController(p, i, d, f);

        // Configuração do telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Inicialização dos motores
        // Motor com encoder
        KR = hardwareMap.get(DcMotorEx.class, "KR"); // Motor sem encoder
        // Configurar direção conforme necessário
        KR.setDirection(DcMotorEx.Direction.FORWARD);
        KR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("PIDF para braço com encoder único inicializado!");
    }

    @Override
    public void loop() {
        // Atualizar valores de PIDF em tempo real
        controller.setPIDF(p, i, d, f);

        // Obter posição do encoder do motor KL
        int posKL = KR.getCurrentPosition();

        // Calcular PID com base na posição do encoder do KL
        double pid = controller.calculate(posKL, target);

        // Calcular Feedforward (baseado no ângulo em graus)
        double ff = Math.cos(Math.toRadians(target * ticks_in_degrees)) * f;

        // Calcular potência final
        double output = pid + ff;
        output = Math.max(-1.0, Math.min(1.0, output));
        double speed = 0.8;

        // Aplicar potência aos motores do braço
        KR.setPower(output * speed);


        // Exibir informações no telemetry
        telemetry.addData("Posição KL (ticks):", posKL);
        telemetry.addData("Alvo (ticks):", target);
        telemetry.addData("Potência Final:", output);
        telemetry.addData("PID:", pid);
        telemetry.addData("Feedforward:", ff);
        telemetry.update();
    }
}
