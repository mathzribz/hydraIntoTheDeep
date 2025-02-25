package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class PIDFKit {

    private static PIDFController controller;

    // Constantes de PIDF e alvo inicial
    public static double p = 0.01, i = 0.0, d = 0;
    public static double f = 0.1; // Feedforward inicial


    // Fator de conversão para graus
    public static final double ticks_in_degrees = 360.0 / 28; // Para Through Bore Encoder da REV

    public static double returnKitPIDF(double target, double posArm) {
        // Inicialização do controlador PIDF
        controller = new PIDFController(p, i, d, f);


        // Atualizar valores de PIDF em tempo real
        controller.setPIDF(p, i, d, f);

        // Calcular PID com base na posição do encoder
        double pid = controller.calculate(posArm, target);

        // Calcular Feedforward
        double ff = Math.cos(Math.toRadians(target * ticks_in_degrees)) * f;

        // Calcular potência final
        double output = pid + ff;
        output = Math.max(-1.0, Math.min(1.0, output));

        return output;

    }
}