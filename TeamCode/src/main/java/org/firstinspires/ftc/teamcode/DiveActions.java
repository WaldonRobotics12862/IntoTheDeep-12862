package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class DiveActions{

    // Lift is the class that will support the lift
    // ITD-86
    public static class Lift  {
        private static DcMotorEx liftLeft;
        private static DcMotorEx liftRight;

        public Lift(HardwareMap hardwareMap) {
            liftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
            liftLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftLeft.setDirection(DcMotorEx.Direction.FORWARD);

            liftRight = hardwareMap.get(DcMotorEx.class, "rightLift");
            liftRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftRight.setDirection(DcMotorEx.Direction.FORWARD);

        }

        public static class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    liftLeft.setPower(Variables.leftUpLift);
                    liftRight.setPower(Variables.rightUpLift);
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
                    liftLeft.setPower(Variables.leftStopLift);
                    liftRight.setPower(Variables.rightStopLift);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }

        public static Action LiftUp() {
            return new LiftUp();
        }

        public static class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    liftLeft.setPower(Variables.leftDownLift);
                    liftRight.setPower(Variables.rightDownLift);
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
                    liftLeft.setPower(Variables.leftDownLift);
                    liftRight.setPower(Variables.rightDownLift);
                    return false;
                }
                // overall, the action powers the lift with negative power until it is under
                // 10 encoder ticks, then powers it off
            }
        }

        public static Action LiftDown() {
            return new LiftDown();
        }
    }

    // SpecimenDelivery is the class that supports the squeeze clamp for specimen delivery
    // ITD-88
    public static class SpecimenDelivery {
        static Servo specimenServo;

        public SpecimenDelivery (HardwareMap hardwareMap) {
            specimenServo = hardwareMap.get(Servo.class, "specimenServo");
        }

        public static class open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                specimenServo.setPosition(Variables.specimenLoose);
                specimenServo.resetDeviceConfigurationForOpMode();
                return false;
            }

        }

        public static Action open() {
            return new open();
        }
        public static class close implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specimenServo.setPosition(Variables.specimenPinch);
                return false;
            }
        }

        public static Action close() {
            return new close();
        }



    }

    // sampleDelivery is the class that supports the basket that will dump a sample
    // ITD-85
    public static class sampleDelivery {
        static Servo bucketServo;

        public sampleDelivery(HardwareMap hardwareMap) {
            bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        }

        //since this is the basket, we probably don't want these actions called open and close servo
        // they probably should be something like 'dump' and 'load' or something.
        public static class load implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                bucketServo.setPosition(Variables.sampleLoad);
                return false;
            }
        }

        public static Action load() {
            return new load();
        }

        public static class dump implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                bucketServo.setPosition(Variables.sampleDump);
                return false;
            }
        }

        public static Action dump() {
            return new dump();
        }
    }

    // LED is the class that supports displaying LEDs to indicate status of the robot
    // ITD-89
    public static class LED {
         RevBlinkinLedDriver blinkinLedDriver;
         RevColorSensorV3 Colorsensor;

         public LED(HardwareMap hardwareMap){
             RevBlinkinLedDriver blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
             RevColorSensorV3 Colorsensor=hardwareMap.get(RevColorSensorV3.class, "sampleColor");
         }

         //public class Light implements Action {
         //    if(Colorsensor) = "red"
         //    Light blinkin = "red")
         //            if(Colorsensor="blue"
         //    Light blinkin "blue")
         //            if(Colorsensor="yellow"
         //    Light blinkin "yellow")
         //}
         //public Action Light(){
         //    return new Light();
         //}
    }

    // Ascend is the class that supports the Ascend actions to lift the hooks up and pull them
    // back in.
    // ITD-84
    public static class Ascend {
         static Servo ascendServo1;
         static Servo ascendServo2;
         static DcMotorEx ascend;

         public Ascend(HardwareMap hardwareMap){
             ascendServo1 = hardwareMap.get(Servo.class, "ascendServo1");
             ascendServo2 = hardwareMap.get(Servo.class, "ascendServo2");
             ascend = hardwareMap.get(DcMotorEx.class, "ascend");
         }

         public static class Deploy implements Action {
             @Override
             public boolean run(@NonNull TelemetryPacket packet) {
                 ascendServo1.setPosition(Variables.ascend1Up);
                 ascendServo2.setPosition(Variables.ascend2Up);
                 return false;
             }
         }
         public static class UnDeploy implements Action   {
             @Override
             public boolean run(@NonNull TelemetryPacket packet){
                 ascend.setPower(Variables.ascend);

                 return false;
             }
         }

         public static Action Deploy(){
             return new Deploy();
        }
    }

    // Intake is the class that supports us pulling in the sample from the field
    // ITD-87
    public static class intake {
        static Servo ExtendIntake;
        static Servo WristIntake;
        static CRServo Sample_ForIntake;

        public intake (HardwareMap hardwareMap) {
            ExtendIntake = hardwareMap.get(Servo.class,"intakeExtend" );
            WristIntake = hardwareMap.get(Servo.class,"intakeWrist" );
            Sample_ForIntake = hardwareMap.get(CRServo.class,"sampleServo" );
        }

        public static class ExtendArm implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ExtendIntake.setPosition(Variables.extendedIntake);
                WristIntake.setPosition(Variables.wristIntaking);
                Sample_ForIntake.setPower(Variables.sampleIntaking);
                return false;
            }
        }
        public static Action ExtendArm(){
             return new ExtendArm();
        }

        public static class RetractArm implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ExtendIntake.setPosition(Variables.retractedIntake);
                WristIntake.setPosition(Variables.wristUp);
                Sample_ForIntake.setPower(Variables.sampleStop);
                return false;
            }
        }
        public static Action RetractArm(){
            return new RetractArm();
        }

        public static class WheelOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ExtendIntake.setPosition(Variables.extendedIntake);
                WristIntake.setPosition(Variables.wristUp);
                Sample_ForIntake.setPower(1);
                return false;
            }

            public boolean stop(@NonNull TelemetryPacket packet) {
                ExtendIntake.setPosition(Variables.retractedIntake);
                WristIntake.setPosition(Variables.wristIntaking);
                Sample_ForIntake.setPower(0);
                return false;
            }

            public class On implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    ExtendIntake.setPosition(Variables.extendedIntake);
                    WristIntake.setPosition(Variables.wristUp);
                    Sample_ForIntake.setPower(-1);
                    return false;
                }
            }
        }
        public static Action WheelOn() {
            return new WheelOn();
        }
    }
}