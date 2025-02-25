

package org.firstinspires.ftc.teamcode.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private DcMotorEx EncoderServoP;
    private TouchSensor mag;
    double speed = 0.8;
    private boolean isManualControl = true;
    boolean Collect = false;
    boolean homingDone = false;
    public PIDFController controllerANG, controllerKIT, controllerArm, controllerServoP;
    final double ticks_in_degrees = 360.0 / 28;
    double MAX_ANGLE = 180;
    final double ticks_in_degrees_Servos = MAX_ANGLE / 8192;


    // Variáveis PIDF para o braço
    public static double angP = 0.01, angI = 0, angD = 0, angF = 0.1;
    public static int targetANG   ;
    public static double speedANG = 1;

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

    private ElapsedTime timer = new ElapsedTime();

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

            if (isManualControl) {
                AngControl();
                KitControl();
                ServosControl();
                ArmControl();
                applyArm_PIDF();
                applyServoP_PIDF();

                Collect = false;
            } else {

                applyAngPIDF();
                applyKitPIDF();
                applyServoP_PIDF();
                AutoFunctions();
                applyArm_PIDF();

            }


            // Atualizar telemetria

            telemetry.addData("Modo", isManualControl ? "Manual" : "Automático");
            telemetry.addData("targetANG", targetANG);
            telemetry.addData("power AR", AR.getPower());
            telemetry.addData("power AL", AL.getPower());
            telemetry.addData("Collect ", Collect);
            telemetry.addData("Ang ticks", AR.getCurrentPosition());
            telemetry.addData("Kit ticks", KR.getCurrentPosition());
            telemetry.addData("servoG", servoG.getPosition());
            telemetry.addData("servoP", servoP.getPosition());
            telemetry.addData(" Arm  ticks ", Arm.getCurrentPosition());
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
        AL.setDirection(DcMotor.Direction.FORWARD);

        KR.setDirection(DcMotor.Direction.FORWARD);

        Arm.setDirection(DcMotor.Direction.REVERSE);

        // Encoders
        RMF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LMF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


// Define valores
        controllerANG.setPIDF(angP, angI, angD, angF);

        controllerKIT.setPIDF(kitP, kitI, kitD, kitF);

        controllerArm.setPIDF(Arm_P, Arm_I, Arm_D, Arm_F );

        controllerServoP.setPIDF(servoP_P, servoP_I, servoP_D, servoP_F );

    }

    public void initAng() {
        if (!homingDone) {
            // Mover o braço lentamente para baixo até o sensor ativar
            while (!mag.isPressed() && opModeIsActive()) {
                AR.setPower(0.2);
                AL.setPower(-0.2);
            }
            AR.setPower(0);
            AL.setPower(0);

            homingDone = true;
        }
        if (mag.isPressed() && homingDone) {
            AR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            targetArm = -50;

        }
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
        RMF.setPower(frontRightPower);
        RMB.setPower(backRightPower);
        LMF.setPower(frontLeftPower);
        LMB.setPower(backLeftPower);

        telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles());

}

    public void AngControl() {
        double LT = gamepad1.left_trigger;
        double RT = gamepad1.right_trigger;
        double maxSpeed = 0.5;
        double tickMax = -2200;
        double currentTicksAng = AR.getCurrentPosition();

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
            servoG.setPosition(0.6);
        } else if (gamepad1.right_bumper) {
            servoG.setPosition(0);
        }

        // Pulso
        if (gamepad1.dpad_up) {
          // targetServoP = 1;
            servoP.setPosition(1);
        }
        else if (gamepad1.dpad_left) {
            // targetServoP = 5;
            servoP.setPosition(0.5);
        }
        else if (gamepad1.dpad_down){
           // targetServoP = 0;
            servoP.setPosition(0);
        }

    }

    public void ArmControl() {

        if (gamepad1.x) {
            targetArm = -25;
        }
        else if (gamepad1.y) {
            targetArm = -5;
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

    }

    public void Collect_Submersible() {


        targetANG = 0; speedANG = 0.5;

        targetKIT = 0; speedKIT = 1.0;

        targetArm = 0; speedArm = 1.0;

        targetServoP = 0;

    }

    public void Collect_Specimen() {


        targetANG = -20; speedANG = 0.5;

        targetKIT = 20; speedKIT = 1.0;

        targetArm = 0; speedArm = 1.0;

        targetServoP = 0;

    }

    public void Deposit_Chamber() {

        targetANG = -0; speedANG = 0.5;

        targetKIT = 0; speedKIT = 1.0;

        targetArm = 0; speedArm = 1.0;

        targetServoP = 0;

    }

    public void Deposit_Basket() {


        targetANG = -0; speedANG = 0.6;

        targetKIT = 0; speedKIT = 0.5;

        targetArm = 0; speedArm = 1.0;

        targetServoP = 0;

    }

    private void applyAngPIDF() {
        if (!isManualControl) {

            // Leitura de posição pelo encoder
            int currentPosition = AR.getCurrentPosition();

            double pid = controllerANG.calculate(currentPosition, targetANG);

            // Calcular Feedforward
            double ff = Math.cos(Math.toRadians(targetANG * ticks_in_degrees)) * angF;

            // Calcular potência final
            double output = pid + ff;
            output = Math.max(-1.0, Math.min(1.0, output));

            // Aplicar potência aos motores do braço
            AR.setPower(output);
            AL.setPower(-output);

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

