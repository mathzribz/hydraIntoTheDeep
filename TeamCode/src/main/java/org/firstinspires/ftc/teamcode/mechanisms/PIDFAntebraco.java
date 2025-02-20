package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;

@Config
public class PIDFAntebraco {

    private static PIDFController controller;

    // Constantes de PIDF e Feedforward
    public static double p = 0.01, i = 0.0, d = 0.001, f = 0.1;

    // Conversão de ticks para graus
    public static final double ticks_in_degrees = 180.0 / 8192.0; // Para um encoder externo de 8192 ticks

    public static double returnAntebracoPIDF(double target, double posAtual) {
        // Inicializar o controlador PIDF
        controller = new PIDFController(p, i, d, f);

        // Atualizar valores de PIDF em tempo real
        controller.setPIDF(p, i, d, f);

        // Calcular PID com base na posição do encoder
        double pid = controller.calculate(posAtual, target);

        // Calcular Feedforward (considerando a gravidade)
        double ff = Math.cos(Math.toRadians(target)) * f;

        // Calcular saída final (limitada entre 0 e 1)
        double output = Math.max(0, Math.min(1, (posAtual / 180.0) + pid + ff));

        return output;
    }
}