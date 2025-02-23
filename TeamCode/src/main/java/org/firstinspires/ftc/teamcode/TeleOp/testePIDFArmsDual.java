
package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class testePIDFArmsDual extends OpMode {

    private PIDFController controller;

    // Constantes de PIDF e alvo inicial
    public static double p = 0.01, i = 0.0, d = 0.000;
    public static double f = 0.1; // Feedforward inicial
    public static int target = 0; // Alvo inicial em ticks

    // Fator de conversão para graus
    public final double ticks_in_degrees = 360.0 / 8192; // Para Through Bore Encoder da REV

    // Motores e Encoder Externo
    private DcMotorEx AR, AL; // Motores do braço
    private DcMotorEx Pivot; // Encoder externo

    @Override
    public void init() {
        // Inicialização do controlador PIDF
        controller = new PIDFController(p, i, d, f);

        // Configuração do telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Inicialização dos motores
        AR = hardwareMap.get(DcMotorEx.class, "AR"); // Braço Direito
        AL = hardwareMap.get(DcMotorEx.class, "AL"); // Braço Esquerdo
        Pivot = hardwareMap.get(DcMotorEx.class, "AR"); // Encoder conectado à porta "AR"

        AR.setDirection(DcMotorEx.Direction.FORWARD);
        AL.setDirection(DcMotorEx.Direction.FORWARD);

        // Configurar Encoder Externo
        Pivot.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // Usado apenas para leitura de posição

    }

    @Override
    public void loop() {
        // Atualizar valores de PIDF em tempo real
        controller.setPIDF(p, i, d, f);

        // Obter a posição do encoder externo (KL)
        int posPivot = AR.getCurrentPosition();

        // Calcular PID com base na posição do encoder
        double pid = controller.calculate(posPivot, target);

        // Calcular Feedforward
        double ff = Math.cos(Math.toRadians(target * ticks_in_degrees)) * f;

        // Calcular potência final
        double output = pid + ff;
        output = Math.max(-1.0, Math.min(1.0, output));
        double speed = 0.7;

        // Aplicar potência aos motores do braço
        AR.setPower(output * speed);
        AL.setPower(output * speed);

        // Exibir informações no telemetry
        telemetry.addData("Posição Encoder (ticks):", posPivot);
        telemetry.addData("Alvo (ticks):", target);
        telemetry.addData("Potência Final:", output);
        telemetry.addData("PID:", pid);
        telemetry.addData("Feedforward:", ff);
        telemetry.update();
    }
}
