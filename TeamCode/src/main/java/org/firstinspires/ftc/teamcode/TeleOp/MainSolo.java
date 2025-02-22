

package org.firstinspires.ftc.teamcode.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Config
@TeleOp
public class MainSolo extends LinearOpMode {
    private DcMotorEx RMF, RMB, LMF, LMB, KR, AR, AL, Arm;
    private Servo servoG, servoP;
    private DcMotorEx EncoderANG, EncoderKIT, EncoderServoP;
    private TouchSensor mag;
    double speed = 0.8;
    private boolean isManualControl = true;
    boolean Collect = false;
    boolean homingDone = false;
    public PIDFController controllerANG, controllerKIT, controllerArm, controllerServoP;
    final double ticks_in_degrees = 360.0 / 8192;
    final double ticks_in_degrees_kit = 360.0 / 28;
    final double ticks_in_degrees_Arm = 360.0 / 288;
    double maxVelocity = 933; // Para 36:1

    double MAX_ANGLE = 180;
    final double ticks_in_degrees_Servos = MAX_ANGLE / 8192;


    // Variáveis PIDF para o braço
    public static double angP = 0.05, angI = 0, angD = 0, angF = 0.001;
    public static int targetANG;
    public static double speedANG = 1;

    // Variáveis PIDF para o Kit Liner
    public static double kitP = 0.03, kitI = 0, kitD = 0, kitF = 0.001;
    public static int targetKIT;
    public static double speedKIT = 1;

    // Variáveis PIDF para o servo Antebraço
    public static double Arm_P = 0.01, Arm_I = 0, Arm_D = 0, Arm_F = 0.1;
    public static int targetArm;
    public static double speedArm = 1;

    // Variáveis PIDF para o servo Pulso
    public static double servoP_P = 0.01, servoP_I = 0, servoP_D = 0, servoP_F = 0.01;
    public static int targetServoP;

    private ElapsedTime timer = new ElapsedTime();
    boolean isBlueAlliance = true; // Troque para false se for Red Alliance
    boolean lastShare = false;

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        initt();

        waitForStart();

        while (opModeIsActive()) {

            initAng();
            loc();

            // Alternância entre os modos ao pressionar o touchpad
            if (gamepad1.touchpad) {
                isManualControl = !isManualControl;
                sleep(300); // Pequeno delay para evitar múltiplos registros do toque
            }

            if (gamepad1.options && !lastShare) { // Detecta apenas a transição do botão
                isBlueAlliance = !isBlueAlliance; // Alterna entre azul e vermelho
            }
            lastShare= gamepad1.options ; // Atualiza o estado anterior do botão

            if (isManualControl) {
                AngControl();
                KitControl();
                ServosControl();
                ArmControl();
                applyKitPIDF();
                applyServoP_PIDF();
                Collect = false;
            } else {
                applyAngPIDF();
                applyKitPIDF();
                applyArm_PIDF();
                applyServoP_PIDF();
                AutoFunctions();
                ServosControl();
            }


            // Atualizar telemetria

            telemetry.addData("Modo", isManualControl ? "Manual" : "Automático");
            telemetry.addData("Collect ", Collect);
            telemetry.addData("Ang ticks", EncoderANG.getCurrentPosition());
            telemetry.addData("Kit ticks", KR.getCurrentPosition());
            telemetry.addData("servoG", servoG.getPosition());
            telemetry.addData("servoP", servoP.getPosition());
            telemetry.addData(" Arm  ticks ", Arm.getCurrentPosition());
            telemetry.addData("Velocidade", speed);
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
        // Ante braço
        Arm = hardwareMap.get(DcMotorEx.class, "Arm"); // porta 3 expension
        // Garra
        servoG = hardwareMap.get(Servo.class, "servoG"); // porta 0 expension
        // Pulso
        servoP = hardwareMap.get(Servo .class, "servoP"); // porta 1 expension
        // Antebraço

        // Through bore Encoders
        EncoderANG = hardwareMap.get(DcMotorEx.class, "AR"); // porta  de control (encoder)
        EncoderKIT = hardwareMap.get(DcMotorEx.class, "KR"); // porta  de control (encoder)
        EncoderServoP = hardwareMap.get(DcMotorEx.class, "LMB"); // porta  de control (encoder)
        // Sensor Magnético
        mag = hardwareMap.get(TouchSensor.class, "mag");

        controllerANG = new PIDFController(angP, angI, angD, angF);
        controllerKIT = new PIDFController(kitP, kitI, kitD, kitF);
        controllerArm = new PIDFController(Arm_P, Arm_I, Arm_D, Arm_F);
        controllerServoP = new PIDFController(servoP_P, servoP_I, servoP_D, servoP_F);

        // Direções dos motores
        RMF.setDirection(DcMotor.Direction.FORWARD);
        RMB.setDirection(DcMotor.Direction.FORWARD);
        LMF.setDirection(DcMotor.Direction.REVERSE);
        LMB.setDirection(DcMotor.Direction.REVERSE);

        AR.setDirection(DcMotor.Direction.REVERSE);
        AL.setDirection(DcMotor.Direction.REVERSE);

        KR.setDirection(DcMotor.Direction.FORWARD);

        Arm.setDirection(DcMotor.Direction.REVERSE);

        // Encoders
        RMF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LMF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        AR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        KR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void initAng() {
        if (!homingDone) {
            // Mover o braço lentamente para baixo até o sensor ativar
            while (!mag.isPressed() && opModeIsActive()) {
                AR.setPower(-0.2);
                AL.setPower(-0.2);
            }
            AR.setPower(0);
            AL.setPower(0);

            homingDone = true;
        }
        if (mag.isPressed() && homingDone) {
            EncoderANG.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }

    public void loc() {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        imu.initialize(parameters);

        // Leitura dos controles do joystick
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Zona morta
        double deadzone = 0.05;
        if (Math.abs(turn) < deadzone) turn = 0;
        if (Math.abs(strafe) < deadzone) strafe = 0;
        if (Math.abs(drive) < deadzone) drive = 0;

        // Identifica a orientação do robô
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Define se está na aliança azul (true) ou vermelha (false)

        // Se estiver na aliança oposta, ajusta a referência
        if (!isBlueAlliance) {
            botHeading += Math.PI;
        }

        // Cálculo de rotação do movimento
        double rotX = (strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading));
        double rotY = (strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading));

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);

        double powerLMF = (rotY + rotX + turn) / denominator;
        double powerLMB = (rotY - rotX + turn) / denominator;
        double powerRMF = (rotY - rotX - turn) / denominator;
        double powerRMB = (rotY + rotX - turn) / denominator;

        // Aplicação das potências
        RMF.setPower(powerRMF * speed);
        RMB.setPower(powerRMB * speed);
        LMF.setPower(powerLMF * speed);
        LMB.setPower(powerLMB * speed);

        telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles());
        telemetry.addData("Alliance", isBlueAlliance ? "Blue" : "Red");
        telemetry.update();

}

    public void AngControl() {
        double LT = gamepad1.left_trigger;
        double RT = gamepad1.right_trigger;
        double maxSpeed = 0.5;
        double tickMax = -2200;
        double currentTicksAng = EncoderANG.getCurrentPosition();

        // Inicializa a potência dos motores
        double armPower = 0;

        if (isManualControl) {// Lógica de controle com fim de curso
            if (currentTicksAng <= tickMax && LT > 0) {
                // Se o braço já atingiu o limite máximo, impede a subida
                armPower = 0; // Não sobe
            } else {
                // Calcula a potência do braço com base nos gatilhos
                armPower = (LT - RT) * maxSpeed;
            }


            // Permite apenas descer caso esteja no limite
            if (currentTicksAng <= tickMax && LT > 0) {
                armPower = RT * maxSpeed; // Só permite descer
            }

            // Aplica a potência calculada nos motores
            AR.setPower(armPower);
            AL.setPower(armPower);
        }
    }

    public void KitControl() {
        double ticksMax = 3250; // Limite superior em ticks
        double ticksMin = -1300; // Limite inferior em ticks
        double kitPower = 1;

        // Leitura da posição do encoder de KL
        int currentTicksKL = KR.getCurrentPosition();

        if (gamepad1.a) {
            if (currentTicksKL < ticksMax) {
                  // Subindo
                KR.setPower(kitPower);
            } else {
                // Bloqueia no limite superior
                KR.setPower(0);
            }
        } else if (gamepad1.b) {
            if (currentTicksKL > ticksMin) {
              // Descendo
                KR.setPower(-kitPower);
            } else {
                 // Bloqueia no limite inferior
                KR.setPower(0);
            }
        } else {
            // Parado quando não há entrada
            KR.setPower(0);
        }

    }

    public void ServosControl() {
        // Garra
        if (gamepad1.left_bumper ) {
            servoG.setPosition(1);
        } else if (gamepad1.right_bumper) {
            servoG.setPosition(0);
        }

        // Pulso

        if (gamepad1.dpad_down) {
            targetServoP = 0;

        } else if (gamepad1.dpad_up) {
           targetServoP = 8;

        }

        // Pulso Manual
        else if (gamepad1.y && isManualControl) {
            targetServoP = 5;
        }

    }

    public void ArmControl() {

        if (gamepad1.dpad_left) {
            targetArm = 10;
        }
        else if (gamepad1.dpad_right) {
            targetArm = 20;
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

        // Coleta
        if (Collect ) {
            if (gamepad1.left_trigger > 0.2){
                targetArm = 10;
            }
            else {
                targetArm = 20;
            }
        }
        telemetry.addData("target Ang", targetANG);
        telemetry.addData("target Kit", targetKIT);
        telemetry.update();
    }

    public void Collect_Submersible() {
        applyAngPIDF();
        applyKitPIDF();

        targetANG = 0; speedANG = 0.5;

        targetKIT = 0; speedKIT = 1.0;

        targetArm = 0; speedArm = 1.0;

        targetServoP = 0;

    }

    public void Collect_Specimen() {
        applyAngPIDF();
        applyKitPIDF();

        targetANG = -0; speedANG = 0.55;

        targetKIT = 0; speedKIT = 1.0;

        targetArm = 0; speedArm = 1.0;

        targetServoP = 0;

    }

    public void Deposit_Chamber() {
        applyAngPIDF();
        applyKitPIDF();

        targetANG = -0; speedANG = 0.5;

        targetKIT = 0; speedKIT = 1.0;

        targetArm = 0; speedArm = 1.0;

        targetServoP = 0;

    }

    public void Deposit_Basket() {
        applyAngPIDF();
        applyKitPIDF();

        targetANG = -0; speedANG = 0.6;

        targetKIT = 0; speedKIT = 0.5;

        targetArm = 0; speedArm = 1.0;

        targetServoP = 0;

    }

    private void applyAngPIDF() {
        if (!isManualControl) {
            // Aplicar PIDF apenas se o controle manual não estiver ativo
            controllerANG.setPIDF(angP, angI, angD, angF);
            int currentPosition = EncoderANG.getCurrentPosition();
            double targetVelocity = controllerANG.calculate(currentPosition, targetANG);

            targetVelocity = Math.max(-maxVelocity, Math.min(maxVelocity, targetVelocity));
           // double ff = Math.cos(Math.toRadians(targetKIT * ticks_in_degrees_kit)) * kitF;


            AR.setVelocity(targetVelocity * speedANG);
            AL.setVelocity(targetVelocity * speedANG);
        }
    }

    private void applyKitPIDF() {
        if (!isManualControl) {
            double RT = 1;
            if(Collect){
                double triggerValue = gamepad1.right_trigger; // Valor de 0 a 1
                int minKIT = 50;
                int maxKIT = 200;

                targetKIT = minKIT + (int) (triggerValue * (maxKIT - minKIT));

               RT = 0.5 + (triggerValue * 0.5);
            }
            // Aplicar PIDF apenas se o controle manual não estiver ativo
            controllerKIT.setPIDF(kitP, kitI, kitD, kitF);
            int currentPosition = KR.getCurrentPosition();

            double targetVelocity = controllerKIT.calculate(currentPosition, targetKIT);

            targetVelocity = Math.max(-maxVelocity, Math.min(maxVelocity, targetVelocity));

            KR.setVelocity(targetVelocity * speedKIT);


        }
    }
    private void applyArm_PIDF() {

            controllerArm.setPIDF(Arm_P, Arm_I, Arm_D, Arm_F );

            int currentPosition = Arm.getCurrentPosition(); // Leitura do encoder externo
            double pid = controllerArm.calculate(currentPosition, targetArm);

            double output = pid * speedArm;
            output = Math.max(-1.0, Math.min(1.0, output));


            Arm.setPower(output );

    }
    private void applyServoP_PIDF() {

            controllerServoP.setPIDF(servoP_P, servoP_I, servoP_D, servoP_F );

            int currentPosition = EncoderServoP.getCurrentPosition(); // Leitura do encoder externo
            double currentAngle = currentPosition * ticks_in_degrees_Servos;
            double pid = controllerServoP.calculate(currentAngle, targetServoP); // Controle PID

            double output = pid ;
            double servoPosition = Math.max(0, Math.min(1, currentAngle / MAX_ANGLE + output));

            servoPosition = Math.max(0, Math.min(1, servoPosition));

            // Aplica posição ao servo
            servoP.setPosition(servoPosition);


        }
    }

