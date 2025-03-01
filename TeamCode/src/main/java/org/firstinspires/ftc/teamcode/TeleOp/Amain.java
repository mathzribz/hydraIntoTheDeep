package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.enumerate;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class Amain extends LinearOpMode {

    private PIDFController controller;

    // Variáveis PIDF para o braço
    public static double p = 0.01, i = 0.0, d = 0.0002;
    public static double f = 0.15; // Feedforward inicial
    public static int target; // Alvo inicial em ticks
    public static double speedANG = 0.8;

    private DcMotorEx RMF, RMB, LMF, LMB, AR, AL, KR, Arm;
    private Servo servoG, servoP;
    private DcMotorEx EncoderServoP, EncoderANG;
    private TouchSensor mag;
    double speed = 0.8;
    private boolean isManualControl = true;
    boolean Collect = false;
    boolean homingDone = false;
    public PIDFController controllerANG, controllerKIT, controllerArm, controllerServoP;
    final double ticks_in_degrees = 360.0 / 28;
    double MAX_ANGLE = 180;
    final double ticks_in_degrees_Servos = MAX_ANGLE / 8192;

    // Variáveis PIDF para o Kit Liner
    public static double kitP = 0.01, kitI = 0, kitD = 0, kitF = 0.1;
    public static int targetKIT ;
    public static double speedKIT = 1;

    // Variáveis PIDF para o servo Antebraço
    public static double Arm_P = 0.01, Arm_I = 0, Arm_D = 0, Arm_F = 0.1;
    public static int targetArm = 0;
    public static double speedArm = 0.5;

    // Variáveis PIDF para o servo Pulso
    public static double servoP_P = 0.01, servoP_I = 0, servoP_D = 0, servoP_F = 0.01;
    public static int targetServoP;

    @Override
    public void runOpMode() throws InterruptedException {
        initt();


        // Inicialização do controlador PIDF
        controller = new PIDFController(p, i, d, f);

        // Configuração do telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Inicialização dos motores

        waitForStart();


        while (opModeIsActive()) {
            if (mag.isPressed() && !homingDone) {
           if (!homingDone) {
               EncoderANG.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               AR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
               AL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
           }
            homingDone = true;
        }

            loc();

            if (isManualControl) {
                KitControl();
                ServosControl();
                applyArm_PIDF();
                applyServoP_PIDF();
                AutoFunctions();

            }
            if (Collect ) {
                if (gamepad1.left_trigger > 0.2) {
                    targetArm = -120;
                } else {
                    targetArm = -90;
                }
            }
            if (gamepad2.y) {
                target = -180;
            }
            if (gamepad2.x) {
                target = 0;
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
                AR.setPower(output * speedANG);
                AL.setPower(-output * speedANG);

            telemetry.addData("Modo", isManualControl ? "Manual" : "Automático");
            telemetry.addData("Velocidade", speed);
            telemetry.addData("targetANG", target);
            telemetry.addData("targetServoP", targetServoP);
            telemetry.addData("targetArm", targetArm);
            telemetry.addData("Collect ", Collect);
            telemetry.addData("homingDone ", homingDone);
            telemetry.addData("Ang ticks", AR.getCurrentPosition());
            telemetry.addData("Kit ticks", KR.getCurrentPosition());
            telemetry.update();
        }


    }
    public void initt() {
        // Motores de movimento
        RMF = hardwareMap.get(DcMotorEx.class, "RMF"); // porta 0 expension
        RMB = hardwareMap.get(DcMotorEx.class, "RMB"); // porta 1 expension
        LMF = hardwareMap.get(DcMotorEx.class, "LMF"); // porta 1 control
        LMB = hardwareMap.get(DcMotorEx.class, "LMB"); // porta 0 control
        // Motores do Angulaçâo
        AR = hardwareMap.get(DcMotorEx.class, "AR"); // porta 2 control
        AL = hardwareMap.get(DcMotorEx.class, "AL"); // porta 3 control
        // Motores do Kit
        KR = hardwareMap.get(DcMotorEx.class, "KR"); // porta 2 expension
        // Antebraço
        Arm = hardwareMap.get(DcMotorEx.class, "Arm"); // porta 3 expension
        // Garra
        servoG = hardwareMap.get(Servo.class, "servoG"); // porta 0 expension
        // Pulso
        servoP = hardwareMap.get(Servo .class, "servoP"); // porta 1 expension
        // Through bore Encoder
        EncoderServoP = hardwareMap.get(DcMotorEx.class, "AL"); // porta  de control (encoder)
        EncoderANG = hardwareMap.get(DcMotorEx.class, "AR"); // porta  de control (encoder)
        // Sensor Magnético
        mag = hardwareMap.get(TouchSensor.class, "mag");

        controllerKIT = new PIDFController(kitP, kitI, kitD, kitF);
        controllerArm = new PIDFController(Arm_P, Arm_I, Arm_D, Arm_F);
        controllerServoP = new PIDFController(servoP_P, servoP_I, servoP_D, servoP_F);

        // Direções dos motores
        RMF.setDirection(DcMotor.Direction.FORWARD);
        RMB.setDirection(DcMotor.Direction.FORWARD);
        LMF.setDirection(DcMotor.Direction.REVERSE);
        LMB.setDirection(DcMotor.Direction.REVERSE);

        AR.setDirection(DcMotorEx.Direction.REVERSE);
        AL.setDirection(DcMotorEx.Direction.FORWARD);

        KR.setDirection(DcMotor.Direction.FORWARD);

        Arm.setDirection(DcMotor.Direction.REVERSE);

        // Encoders
        RMF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LMF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        AR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



// Define valores

        controllerKIT.setPIDF(kitP, kitI, kitD, kitF);

        controllerArm.setPIDF(Arm_P, Arm_I, Arm_D, Arm_F );

        controllerServoP.setPIDF(servoP_P, servoP_I, servoP_D, servoP_F );

    }

    public void loc() {

        // Recupera o IMU do hardwareMap
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Define os parâmetros de orientação do Hub no robô
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        // Inicializa o IMU com os parâmetros definidos
        imu.initialize(parameters);

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.dpad_right) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Compensação do strafe
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Aplicar potências
        RMF.setPower(frontRightPower * speed);
        RMB.setPower(backRightPower * speed) ;
        LMF.setPower(frontLeftPower  * speed);
        LMB.setPower(backLeftPower  * speed);

        if (gamepad1.share){
            speed = 0.85;
        }
        if (gamepad1.options){
            speed = 0.65;
        }

        telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles());

    }
    public void AngControl() {
        double LT = gamepad2.left_trigger;
        double RT = gamepad2.right_trigger;
        double maxSpeed = 0.5;
        double tickMax = -3000;
        double currentTicksAng = EncoderANG.getCurrentPosition();

        // Inicializa a potência dos motores
        double armPower ;

        if (isManualControl) {// Lógica de controle com fim de curso
            if (currentTicksAng <= tickMax && LT > 0) {
                // Se o braço já atingiu o limite máximo, impede a subida
                armPower = 0; // Não sobe
            } else {
                // Calcula a potência do braço com base nos gatilhos
                armPower = (RT - LT) * maxSpeed;
            }


            // Permite apenas descer caso esteja no limite
            if (currentTicksAng <= tickMax && LT > 0) {
                armPower = RT * maxSpeed; // Só permite descer
            }

            // Aplica a potência calculada nos motores
            AR.setPower(armPower);
            AL.setPower(-armPower);
        }
        if(mag.isPressed() && RT > 0 ) {
            AR.setPower(0);
            AL.setPower(0);
        }
        if (mag.isPressed()) {
            EncoderANG.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void KitControl() {
        double ticksMax = 2250; // Limite superior em ticks
        double ticksMin = -0; // Limite inferior em ticks
        double kitPower = 1;

        // Leitura da posição do encoder de KL
        int currentTicksKL = KR.getCurrentPosition();
        if ( !Collect ) {
            if (gamepad2.right_bumper) {

                // Subindo
                KR.setPower(kitPower);

            } else if (gamepad2.left_bumper) {

                // Descendo
                KR.setPower(-kitPower);
            } else {
                // Parado quando não há entrada
                KR.setPower(0);
            }
        }
        if (Collect){
            KR.setPower(gamepad1.right_trigger);
        }

    }
    public void ServosControl() {

        // Garra

        if (gamepad1.left_bumper ) {
            servoG.setPosition(0.6);
        } else if (gamepad1.right_bumper) {
            servoG.setPosition(0);
        }

        // Pulso
        if (gamepad1.dpad_up) {
             targetServoP = 20;
        }
        else if (gamepad1.dpad_left) {
             targetServoP = 10;

        }
        else if (gamepad1.dpad_down){
             targetServoP = -5;

        }

    }

    public void AutoFunctions() {
        // Ang Max = -240 Ang Min = 0
        // Kit Max = 290 Kit min = 0


        // Coleta (Submersível)
        if (gamepad1.a) {
            Collect = true;
            Collect_Submersible();
        }

        // Coleta (Specimen)
        if (gamepad1.x) {
            Collect = false;

            Collect_Specimen();
        }

        // Depositar(Chamber)
        if (gamepad1.y) {
            Collect = false;

            Deposit_Chamber();

        }

        // Depositar(Basket)
        if (gamepad1.b) {
            Collect = false;

            Deposit_Basket();
        }
    }

    public void Collect_Submersible() {


       // target = 0; speedANG = 0.5;

       // targetKIT = 0; speedKIT = 1.0;

        targetArm = -90; speedArm = 0.5;

        targetServoP = -5;

    }

    public void Collect_Specimen() {


       // target = 0; speedANG = 0.5;

        //targetKIT = -155; speedKIT = 1.0;

        targetArm = -70; speedArm = 0.5;

        targetServoP = 3;

    }

    public void Deposit_Chamber() {

       // target = -200; speedANG = 0.5;

      //  targetKIT = 0; speedKIT = 1.0;

        targetServoP = 35;

        targetArm = -110; speedArm = 0.5;

    }

    public void Deposit_Basket() {


       // targetANG = 0; speedANG = 0.6;

       // targetKIT = 0; speedKIT = 0.5;

        targetArm = -110; speedArm = 0.5;

        targetServoP = 10;

    }
    private void applyKitPIDF() {
        if (!isManualControl) {
            // Aplicar PIDF apenas se o controle manual não estiver ativo
            int currentPosition = KR.getCurrentPosition();

            double pid = controllerKIT.calculate(currentPosition, targetKIT);

            // Calcular Feedforward
            double ff = Math.cos(Math.toRadians(targetKIT * ticks_in_degrees)) * kitF;

            // Calcular potência final
            double output = pid + ff;
            output = Math.max(-1.0, Math.min(1.0, output));

            // Aplicar potência aos motores do braço
            KR.setPower(output * speedKIT);

        }
    }
    private void applyArm_PIDF() {
        int currentPosition = Arm.getCurrentPosition(); // Leitura do encoder externo
        double pid = controllerArm.calculate(currentPosition, targetArm);

        double ff = Math.cos(Math.toRadians(targetArm * ticks_in_degrees)) * Arm_F;

        // Calcular potência final
        double output = pid + ff;
        output = Math.max(-1.0, Math.min(1.0, output));


        Arm.setPower(output * speedArm);

    }
    private void applyServoP_PIDF() {

        int currentPosition = EncoderServoP.getCurrentPosition(); // Leitura do encoder externo
        double currentAngle = currentPosition * ticks_in_degrees_Servos;
        double pid = controllerServoP.calculate(currentAngle, targetServoP); // Controle PID

        double servoPosition = Math.max(0, Math.min(1, currentAngle / MAX_ANGLE + pid));

        servoPosition = Math.max(0, Math.min(1, servoPosition));

        // Aplica posição ao servo
        servoP.setPosition(servoPosition);


    }
}