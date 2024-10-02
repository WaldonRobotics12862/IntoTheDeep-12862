package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import kotlin.OptionalExpectation;

public class DiveActions {
     public class Lift {
        private DcMotorEx liftLeft;
        private DcMotorEx liftRight;

        public Lift(HardwareMap hardwareMap) {
            liftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
            liftLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftLeft.setDirection(DcMotorEx.Direction.FORWARD);

            liftRight = hardwareMap.get(DcMotorEx.class, "rightLift");
            liftRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftRight.setDirection(DcMotorEx.Direction.FORWARD);

        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    liftLeft.setPower(0.8);
                    liftRight.setPower(0.8);
                    initialized = true;
                }

                // checks lift's current position
                double posL = liftLeft.getCurrentPosition();
                double posR = liftRight.getCurrentPosition();
                packet.put("liftPos", posL);
                if (posR < 3000.0 & posL < 3000.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    liftLeft.setPower(0);
                    liftRight.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }

        public Action LiftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    liftLeft.setPower(-0.8);
                    liftRight.setPower(-0.8);
                    initialized = true;
                }

                // checks lift's current position
                double posL = liftLeft.getCurrentPosition();
                double posR = liftRight.getCurrentPosition();
                packet.put("liftPos", posL);
                if (posR > 10.0 & posL > 10.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    liftLeft.setPower(0);
                    liftRight.setPower(0);
                    return false;
                }
                // overall, the action powers the lift with negative power until it is under
                // 10 encoder ticks, then powers it off
            }
        }

        public Action LiftDown() {
            return new LiftDown();
        }
    }

    public class specimen {
        public specimen(HardwareMap hardwareMap) {
           <>
        }

        public class open implements Action {

        }

        public Action open() {
            return new open;

            public class SampleDelivery {
                public sample()
            }


        }

    }

    public class sampleDelivery {
        Servo sampleServo;

        public sampleDelivery(HardwareMap hardwareMap) {
            sampleServo = hardwareMap.get(Servo.class, "sampleServo");
        }

        public class openServo implements Action {
            sampleServo.setPosition(0.0);
        }

        public Action openServo() {
            return new openServo();
        }

        public class closeServo implements Action {
            sampleServo.setPosition(1.0);
        }

        public Action closeServo() {
            return new closeServo();
        }
    }

    public class LED {
         RevBlinkinLedDriver blinkinLedDriver;
         RevColorSensorV3 Colorsensor;

         public LED(HardwareMap hardwareMap){
             RevBlinkinLedDriver blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
             RevColorSensorV3 Colorsensor=hardwareMap.get(RevColorSensorV3.class, "sampleColor");
         }

         public class Light implements Action {
             if(Colorsensor ="red"
             Light blinkin "red")
                     if(Colorsensor="blue"
             Light blinkin "blue")
                     if(Colorsensor="yellow"
             Light blinkin "yellow")
         }
         public Action Light(){
             return new Light();
         }
    }

    public class Ascend{
         Servo ascendServo1;
         Servo ascendServo2;

         public Ascend(HardwareMap hardwareMap){
             Servo ascendServo1 = hardwareMap.get(Servo.class, "ascendServo1");
             Servo ascendServo2 = hardwareMap.get(Servo.class, "ascendServo2");
         }

         public class Ascend implements Action{
             ascendServo1.setPosition(0);
             ascendServo2.setPosition(0);
         }

         public Action Ascend(){
             return new Ascend();
        }
    }

    public class intake {
        public intake (HardwareMap hardwareMap) {
            Servo ExtendIntake = hardwareMap.get(Servo.class,"intakeExtend" );
            Servo WristIntake = hardwareMap.get(Servo.class,"intakeWrist" );
            Servo Sample_ForIntake = hardwareMap.get(Servo.class,"sampleServo" );
        }
        public class Open Implaments Action{
            ExtendIntake.setPosition(0);
            WristIntake.setPosition(0);
            Sample_ForIntake.serPosition(0);
        }
        public Action Open(){return new Open}
    }
}