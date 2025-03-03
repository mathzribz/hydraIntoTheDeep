
package org.firstinspires.ftc.teamcode.mechanisms;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class subsystems {

    public DcMotorEx KR, AR, AL, Arm, Pivot, extend, encoderA, encoderP;
    public Servo servoG, servoP;

    public static double CLAW_OPEN = 0;
    public static double CLAW_CLOSE = 0.6;

    public class Claw {
        public Claw(HardwareMap hardwareMap) {
            servoG = hardwareMap.get(Servo.class, "servoG");
            servoG.setDirection(Servo.Direction.FORWARD);
        }
        public class ClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servoG.setPosition(CLAW_OPEN);
                return false;
            }
        }
        public class ClawClose implements Action {
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                servoG.setPosition(CLAW_CLOSE);
                return false;
            }
        }
    }

    // Ang
    public class Ang {
        public int setPosition;

        public Ang(HardwareMap hardwareMap) {

            AR = hardwareMap.get(DcMotorEx.class, "AR");
            AR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            AR.setDirection(DcMotorSimple.Direction.FORWARD);

            AL = hardwareMap.get(DcMotorEx.class, "AL");
            AL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            AL.setDirection(DcMotorSimple.Direction.FORWARD);



        }

        public class updatePID implements Action {
            public updatePID() {
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                AR.setPower(PIDFAng.returnArmPIDF(setPosition, AR.getCurrentPosition()));
                AL.setPower(PIDFAng.returnArmPIDF(setPosition, AR.getCurrentPosition()));
                return true;
            }
        }

        public Action UpdatePID_Ang() {
            return new updatePID();
        }

        public class setPosition implements Action {
            int set;

            public setPosition(int position) {
                set = position;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setPosition = set;
                return false;
            }
        }

        public Action SetPosition(int pos) {
            return new setPosition(pos);
        }

    }

    // Kit
    public class Kit {
        public int setPosition;

        public Kit(HardwareMap hardwareMap) {

            KR = hardwareMap.get(DcMotorEx.class, "KR");
            KR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            KR.setDirection(DcMotorSimple.Direction.FORWARD);

        }

        public class updatePID implements Action {
            public updatePID() {
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                KR.setPower(PIDFKit.returnKitPIDF(setPosition, KR.getCurrentPosition()));
                return true;
            }
        }

        public Action UpdatePID_Kit() {
            return new updatePID();
        }

        public class setPosition implements Action {
            int set;

            public setPosition(int position) {
                set = position;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setPosition = set;
                return false;
            }
        }

        public Action SetPosition(int pos) {
            return new setPosition(pos);
        }
    }

    public class Antebraco {
        public int setPosition;
        public Antebraco(HardwareMap hardwareMap) {
            Arm = hardwareMap.get(DcMotorEx.class, "Arm");
            Arm.setDirection(DcMotorEx.Direction.REVERSE);
        }
        // Ação para atualizar a posição do antebraço usando PIDF
        public class updatePID implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Arm.setPower(PIDFKit.returnKitPIDF(setPosition, Arm.getCurrentPosition()));
                return true;
            }
        }
        public Action UpdatePID_Antebraço() {
            return new updatePID();
        }
        public class setPosition implements Action {
            int set;
            public setPosition(int position) {
                set = position;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                setPosition = set;
                return false;
            }
        }
    public Action SetPosition(int pos) {
        return new setPosition(pos);
    }
}



    public class Pulso {

        public int targetPosition = 90; // Posição alvo inicial (90°)

        public Pulso(HardwareMap hardwareMap) {
            servoP = hardwareMap.get(Servo.class, "servoP");
            servoP.setDirection(Servo.Direction.FORWARD);
            encoderP = hardwareMap.get(DcMotorEx.class, "AL");

        }

        // Ação para atualizar a posição do antebraço usando PIDF
        public class updatePID implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int currentPosition = encoderP.getCurrentPosition();
                double currentAngle = currentPosition * PIDFPulso.ticks_in_degrees; // Converte ticks para graus
                double servoPosition = PIDFPulso.returnPulsoIDF(targetPosition, currentAngle);

                servoP.setPosition(servoPosition);


                return true;
            }
        }

        public Action UpdatePID_Pulso() {
            return new updatePID();
        }

        // Classe para definir um novo target
        public class setPosition implements Action {
            int target;

            public setPosition(int position) {
                target = position;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetPosition = target;
                return false;
            }
        }

        public Action SetPosition(int pos) {
            return new setPosition(pos);
        }


    }
}