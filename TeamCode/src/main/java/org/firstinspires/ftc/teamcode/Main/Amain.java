
package org.firstinspires.ftc.teamcode.Main;

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
    private DcMotorEx RMF, RMB, LMF, LMB, AR, AL, KR, Arm;
    private Servo servoG, servoP;
    private DcMotorEx EncoderServoP, EncoderANG;
    private TouchSensor mag, toc;
    double speed = 0.8;
    double ticksMaxKit = 2250;
    double tickMaxAng = -4000;
    private static final double DEAD_ZONE = 0.1;

    // booleans
    boolean op = true;
    boolean Collect, DepositSpecimen, DepositBasket = false;
    boolean homingDone = false;
    boolean extendKit = false;

    // PIDFS
    public PIDFController controllerArm, controllerServoP;
    final double ticks_in_degrees = 360.0 / 28;

    // Variáveis PIDF para o servo Antebraço
    public static double Arm_P = 0.01, Arm_I = 0, Arm_D = 0, Arm_F = 0.1;
    public static int targetArm = 0;
    public static double speedArm = 0.5;

    // Variáveis PIDF para o servo Pulso
    public static double servoP_P = 0.01, servoP_I = 0, servoP_D = 0, servoP_F = 0.01;
    public static int targetServoP;
    double MAX_ANGLE = 180;
    final double ticks_in_degrees_Servos = MAX_ANGLE / 8192;

    @Override
    public void runOpMode() throws InterruptedException {

        // Configuração do telemetry dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Inicialização
        initt();

        waitForStart();

        while (opModeIsActive()) {
            if (op) {
                Homing();
                loc();


                // Controle dos sistemas (Manualmente) com limatadores
                KitControl();
                AngControl();
                ServosControl();
                // PIDFs
                applyArm_PIDF();
                applyServoP_PIDF();
                // Pulso e Antebraço automáticos
                AutoFunctions();
                // Coletar e Depósitar
                Collects();
                Deposits();

            }

            telemetry.addData("Ang ticks", AR.getCurrentPosition());
            telemetry.addData("Kit ticks", KR.getCurrentPosition());
            telemetry.addData("targetServoP", targetServoP);
            telemetry.addData("targetArm", targetArm);
            telemetry.addData("Velocidade", speed);
            telemetry.addData("Collect ", Collect);
            telemetry.addData("Extend Kit ", extendKit);
            telemetry.addData("homingDone ", homingDone);
            telemetry.addData("pressionado ", toc.isPressed());
            telemetry.addData("DepositSpecimen ", DepositSpecimen);
            telemetry.addData("DepositBasket ", DepositBasket);
            telemetry.update();
        }

    }
    public void initt() {
        // Motores Locomoção
        RMF = hardwareMap.get(DcMotorEx.class, "RMF"); // porta 0 expension
        RMB = hardwareMap.get(DcMotorEx.class, "RMB"); // porta 1 expension
        LMF = hardwareMap.get(DcMotorEx.class, "LMF"); // porta 1 control
        LMB = hardwareMap.get(DcMotorEx.class, "LMB"); // porta 0 control
        // Motores Angulaçâo
        AR = hardwareMap.get(DcMotorEx.class, "AR"); // porta 2 control
        AL = hardwareMap.get(DcMotorEx.class, "AL"); // porta 3 control
        // Motores Kit
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
        mag = hardwareMap.get(TouchSensor.class, "mag"); // porta 0 digial control
        // Sensor de Toque
        toc = hardwareMap.get(TouchSensor.class, "toc"); // porta 0 digial expension

        // Inicializar PIDFS
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



        // Define valores
        controllerArm.setPIDF(Arm_P, Arm_I, Arm_D, Arm_F );

        controllerServoP.setPIDF(servoP_P, servoP_I, servoP_D, servoP_F );

    }

    public void Homing() {
        if (!homingDone) {
            long startTime = System.currentTimeMillis(); // Marca o tempo inicial

            // Mover AR e AL para baixo até o sensor 'mag' ser acionado ou timeout (3s)
            while (!mag.isPressed() && opModeIsActive()) {
                if (System.currentTimeMillis() - startTime > 3000) { // Timeout de 3s
                    AR.setPower(0);
                    AL.setPower(0);
                    telemetry.addData("Erro", "Timeout no Homing de AR e AL");
                    telemetry.update();
                    break;
                }
                AR.setPower(0.2);
                AL.setPower(-0.2);
            }
            AR.setPower(0);
            AL.setPower(0);

            startTime = System.currentTimeMillis(); // Reinicia o tempo para KR

            // Mover KR para baixo até o sensor 'toc' ser acionado ou timeout (3s)
            while (!toc.isPressed() && opModeIsActive()) {
                if (System.currentTimeMillis() - startTime > 3000) { // Timeout de 3s
                    KR.setPower(0);
                    telemetry.addData("Erro", "Timeout no Homing de KR");
                    telemetry.update();
                    break;
                }
                KR.setPower(-0.3);
            }
            KR.setPower(0);

            // Marcar que o homing foi concluído
            homingDone = true;
        }

        // Resetar encoders quando sensores forem pressionados
        if (homingDone) {
            if (mag.isPressed()) {
                AR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if (toc.isPressed()) {
                KR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }

    public void Collects() {
        if (Collect ) {
            if (gamepad1.left_trigger > 0.2) {
                targetArm = -125;
            } else {
                targetArm = -90;
            }

            ticksMaxKit = 10000;
/*
            if (gamepad1.right_trigger > 0.1 && KR.getCurrentPosition() < ticksMaxKit) {
                extendKit = true;
                KR.setPower(gamepad1.right_trigger);
            }
            else if (gamepad1.right_trigger < 0.1) {
                extendKit = false;
            }
            else if (!extendKit && !toc.isPressed()) {
                KR.setPower(-0.8);
            }
            else if (toc.isPressed()) {
                KR.setPower(0);
                KR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
*/
            double kitPower = 1;
            if (gamepad1.dpad_up ) {
                KR.setPower(kitPower); // Subindo
            }
            // Descer com o bumper direito e respeitar o sensor de toque
            else if (gamepad1.dpad_down ) {
                KR.setPower(-kitPower); // Descendo
            }
            // Parar o motor caso não esteja pressionando os botões ou atingiu limites
            else {
                KR.setPower(0);
            }

        }
    }
    public void Deposits() {
        if (DepositSpecimen){
            ticksMaxKit = 10000; // ?
            if (gamepad1.y) {
                targetArm = -55;
            } else {
                targetArm = -110;
            }
        }
        if (DepositBasket){
            ticksMaxKit = 100000; // ?
            if (gamepad1.b) {
                targetArm = -95;
            } else {
                targetArm = -120;
            }
        }


    }
    private double applyDeadZone(double value) {
        return (Math.abs(value) > DEAD_ZONE) ? value : 0.0;
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
        if (gamepad1.dpad_right) {
            imu.resetYaw();
        }


        double rawX = gamepad1.left_stick_x;
        double rawY = -gamepad1.left_stick_y; // Inverte Y pois no joystick o eixo positivo é para baixo
        double rawTurn = gamepad1.right_stick_x;

        // Aplica a zona morta
        double x = applyDeadZone(rawX);
        double y = applyDeadZone(rawY);
        double turn = applyDeadZone(rawTurn);

        // Captura o ângulo de rotação do robô
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Converte os comandos do joystick para coordenadas relativas ao campo
        double fieldX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double fieldY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // Compensação do strafe
        fieldX = fieldX * 1.1; // testar

        double denominator = Math.max(Math.abs(fieldY) + Math.abs(fieldX) + Math.abs(turn), 1);

        double frontLeftPower = (fieldY + fieldX + turn) / denominator;
        double backLeftPower = (fieldY - fieldX + turn) / denominator;
        double frontRightPower = (fieldY - fieldX - turn) / denominator;
        double backRightPower = (fieldY + fieldX - turn) / denominator;

        // Aplicar potências
        RMF.setPower(frontRightPower * speed);
        RMB.setPower(backRightPower * speed) ;
        LMF.setPower(frontLeftPower  * speed);
        LMB.setPower(backLeftPower  * speed);


        telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles());
    }
    public void AngControl() {
        double LT = gamepad2.left_trigger;
        double RT = gamepad2.right_trigger;
        double maxSpeed = 0.75;
        double currentTicksAng = EncoderANG.getCurrentPosition();

        // Inicializa a potência dos motores
        double armPower ;

        // Lógica de controle com fim de curso
            if (currentTicksAng <= tickMaxAng && LT > 0) {
                // Se o braço já atingiu o limite máximo, impede a subida
                armPower = 0; // Não sobe
            } else {
                // Calcula a potência do braço com base nos gatilhos
                armPower = (RT - LT) * maxSpeed;
            }

            // Permite apenas descer caso esteja no limite
            if (currentTicksAng <= tickMaxAng && LT > 0) {
                armPower = RT * maxSpeed; // Só permite descer
            }

            // Aplica a potência calculada nos motores
            AR.setPower(armPower);
            AL.setPower(-armPower);

        if(mag.isPressed() && RT > 0 ) {
            AR.setPower(0);
            AL.setPower(0);
            AR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }
    public void KitControl() {
        if (!Collect) {
            double kitPower = 1;
            int currentTicksKL = KR.getCurrentPosition();

            // Subir com o bumper esquerdo e respeitar o limite superior
            if (gamepad2.right_bumper ) {
                KR.setPower(kitPower); // Subindo
            }
            // Descer com o bumper direito e respeitar o sensor de toque
            else if (gamepad2.left_bumper && !toc.isPressed()) {
                KR.setPower(-kitPower); // Descendo
            } else if (gamepad2.left_bumper && currentTicksKL >= ticksMaxKit) {
                KR.setPower(-kitPower);
            }
            // Parar o motor caso não esteja pressionando os botões ou atingiu limites
            else {
                KR.setPower(0);
            }
        }
            // Se o sensor de toque for acionado, resetar o encoder
            if (toc.isPressed()) {
                KR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

    }




    public void ServosControl() {

        // Garra
        if (gamepad1.left_bumper ) {
            servoG.setPosition(0.6);
        } else if (gamepad1.right_bumper) {
            servoG.setPosition(0);
        }


    }

    public void AutoFunctions() {

        // Coleta (Submersível)
        if (gamepad1.a) {
            Collect = true;
            DepositSpecimen = false;
            DepositBasket = false;

            Collect_Submersible();
        }

        // Coleta (Specimen)
        if (gamepad1.x) {
            Collect = false;
            DepositSpecimen = false;
            DepositBasket = false;

            Collect_Specimen();
        }

        // Depositar(Chamber)
        if (gamepad1.y) {
            Collect = false;
            DepositSpecimen = true;
            DepositBasket = false;

            Deposit_Chamber();
        }

        // Depositar(Basket)
        if (gamepad1.b) {
            Collect = false;
            DepositSpecimen = false;
            DepositBasket = true;

            Deposit_Basket();
        }
    }

    public void Collect_Submersible() {
        speed = 0.825;


        targetArm = -90; speedArm = 0.5;

        targetServoP = -4;

    }

    public void Collect_Specimen() {

        speed = 0.65;

        targetArm = -85; speedArm = 0.5;

        targetServoP = 13;

    }

    public void Deposit_Chamber() {
        speed = 0.8;

        targetServoP = 42;

        targetArm = -55; speedArm = 0.5;

    }

    public void Deposit_Basket() {

        speed = 0.65;

        targetArm = -95; speedArm = 0.5;

        targetServoP = 10;

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