package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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
            liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            liftRight = hardwareMap.get(DcMotorEx.class, "rightLift");
            liftRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftRight.setDirection(DcMotorEx.Direction.REVERSE);
            liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        public static class LiftToHeight implements Action {
            private Integer height;
            public LiftToHeight(Integer height){
                this.height = height;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftLeft.setTargetPosition(height);
                liftRight.setTargetPosition(height);
                liftLeft.setPower(1);
                liftRight.setPower(1);
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (liftLeft.isBusy() || liftRight.isBusy()){
                    return true;
                } else {
                    return false;
                }
            }
        }
        public static Action liftToHeight(Integer height){
            return new LiftToHeight(height);
        }

        public static class LiftToHighBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Integer height = -2250;
                liftLeft.setTargetPosition(height);
                liftRight.setTargetPosition(height);
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftLeft.setPower(1);
                liftRight.setPower(1);

                packet.addLine(String.valueOf(liftLeft.getCurrentPosition()));

                if (liftLeft.getCurrentPosition() < .9 * height && liftLeft.getCurrentPosition() > 1.1 * height){
                    return false;
                }else {
                    return true;
                }
            }
        }
        public static Action liftToHighBasket() {
            return new LiftToHighBasket();
        }

        public static class LiftFullDown implements Action {
            long downStarted = -1;
            public LiftFullDown(long downStarted){
                this.downStarted = downStarted;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Integer height = 0;
                liftLeft.setTargetPosition(height);
                liftRight.setTargetPosition(height);
                liftLeft.setPower(-1);
                liftRight.setPower(-1);
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (System.currentTimeMillis() - downStarted > 1500) {
                    liftLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    liftRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    return false;
                } else if (liftLeft.isBusy() || liftRight.isBusy()){
                    return true;
                } else {
                    liftLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    liftRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    return false;
                }
            }
        }
        public static Action liftFullDown(long downStarted) {
            return new LiftFullDown(downStarted);
        }

        public static class AutonDown implements Action {
            long downStarted = -1;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Integer height = 0;
                liftLeft.setTargetPosition(height);
                liftRight.setTargetPosition(height);
                liftLeft.setPower(-1);
                liftRight.setPower(-1);
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

               if (liftLeft.getCurrentPosition() > -10 && liftLeft.getCurrentPosition() < 10){
                   liftLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                   liftRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                   return true;
                } else {
                    return false;
                }
            }
        }
        public static Action autonDown() {
            return new AutonDown();
        }

        public static class LiftToHighChamber implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Integer height = -1200;
                liftLeft.setTargetPosition(height);
                liftRight.setTargetPosition(height);
                liftLeft.setPower(1);
                liftRight.setPower(1);
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (liftLeft.isBusy() || liftRight.isBusy()){
                    return true;
                }else {
                    return false;
                }
            }
        }
        public static Action liftToHighChamber() {
            return new LiftToHighChamber();
        }

        public static class DeliverHighChamber implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Integer height = -1000;
                liftLeft.setTargetPosition(height);
                liftRight.setTargetPosition(height);
                liftLeft.setPower(1);
                liftRight.setPower(1);
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (liftLeft.isBusy() || liftRight.isBusy()){
                    return true;
                }else {
                    return false;
                }
            }
        }
        public static Action deliverHighChamber() {
            return new DeliverHighChamber();
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
                if (posL < 2500.0) {
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
                if (posL > 10.0) {
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
                return false;
            }

        }

        public static Action open() {
            return new open();
        }
        public static class close implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.addLine(String.valueOf(specimenServo.getPosition()));
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
    public static class SampleDelivery {
        static Servo bucketServo;

        public SampleDelivery(HardwareMap hardwareMap) {
            bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        }

        //since this is the basket, we probably don't want these actions called open and close servo
        // they probably should be something like 'dump' and 'load' or something.
        public static class Load implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                bucketServo.setPosition(Variables.sampleLoad);
                return false;
            }
        }

        public static Action load() {
            return new Load();
        }

        public static class Dump implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                bucketServo.setPosition(Variables.sampleDump);
                return false;
            }
        }

        public static Action dump() {
            return new Dump();
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
        public static Action deploy(){
            return new Deploy();
        }

        public static class PullUp implements Action   {
             @Override
             public boolean run(@NonNull TelemetryPacket packet){
                 ascend.setPower(Variables.ascend);

                 return false;
             }
         }
        public static Action pullUp(){
             return new PullUp();
         }
    }

    // Intake is the class that supports us pulling in the sample from the field
    // ITD-87
    public static class Intake {
        static Servo ExtendIntake;
        static Servo WristIntake;
        static CRServo Sample_ForIntake;

        public Intake (HardwareMap hardwareMap) {
            ExtendIntake = hardwareMap.get(Servo.class,"intakeExtend" );
            WristIntake = hardwareMap.get(Servo.class,"intakeWrist" );
            Sample_ForIntake = hardwareMap.get(CRServo.class,"sampleServo" );
        }

        public static class DebugAction implements Action {
            private String message;
            private Telemetry telemetry;
            public DebugAction(Telemetry telemetry, String message) {
                this.telemetry = telemetry;
                this.message = message;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetry.addData("Debug", message);
                telemetry.update();

                System.out.println(message);
                return false;
            }
        }

        //ExtendArm does:extends the arm, lowers wrist and turns on the intake wheel
        public static class ExtendArm implements Action{
            private Telemetry telemetry;
            public ExtendArm(Telemetry telemetry) {
                this.telemetry = telemetry;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ExtendIntake.setPosition(Variables.extendedIntake);
                WristIntake.setPosition(Variables.wristDown);
                if (ExtendIntake.getPosition() != Variables.extendedIntake) {
                    return true;
                }
                if (WristIntake.getPosition() != Variables.wristDown) {
                    return true;
                }
                //Sample_ForIntake.setPower(Variables.sampleIntaking);
                return false;
            }
        }
        public static Action extendArm(Telemetry telemetry){
             return new ExtendArm(telemetry);
        }

        //RetractArm does: pulls arm in, pulls the wrist up and stops the wheel
        public static class RetractArm implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ExtendIntake.setPosition(Variables.retractedIntake);
                WristIntake.setPosition(Variables.wristUp);
                if (ExtendIntake.getPosition() != Variables.retractedIntake) {
                    return true;
                }
                if (WristIntake.getPosition() != Variables.wristUp) {
                    return true;
                }
                Sample_ForIntake.setPower(Variables.sampleStop);
                return false;
            }
        }
        public static Action retractArm(){
            return new RetractArm();
        }

        //WheelOn does: just turns on the wheel

        private static class WheelOn implements Action {
            long startTime = -1;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime != -1) {
                    if (System.currentTimeMillis() - startTime < 5000) {
                        return true;
                    }
                    startTime = -1;
                    return false;
                }
                Sample_ForIntake.setDirection(CRServo.Direction.FORWARD);
                Sample_ForIntake.setPower(Variables.sampleIntaking);
                startTime = System.currentTimeMillis();
                return true;
            }
        }
        protected static Action wheelOn() {
            return new WheelOn();
        }

        //WheelOn0 is different than just WheelOn() in that it does not wait for x amount of time. This will be used in Teleop.
        private static class WheelOn0 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Sample_ForIntake.setDirection(CRServo.Direction.FORWARD);
                Sample_ForIntake.setPower(Variables.sampleIntaking);
                return false;
            }
        }
        protected static Action wheelOn0() {
            return new WheelOn0();
        }

        //RevWheel does: just turns on the wheel
        public static class RevWheel implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Sample_ForIntake.setDirection(CRServo.Direction.REVERSE);
                Sample_ForIntake.setPower(Variables.sampleEject);
                return false;
            }
        }
        public static Action RevWheel() {
            return new RevWheel();
        }

        //Stop does: stop the wheel from turning
        public static class Stop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Sample_ForIntake.setPower(0);
                return false;
            }
        }
        public static Action Stop() {
            return new Stop();
        }

        public static class WristUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                WristIntake.setPosition(Variables.wristUp);
                return false;
            }
        }
        public static Action WristUp(){
            return new WristUp();
        }

        public static class Wristdown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                WristIntake.setPosition(Variables.wristDown);
                return false;
            }
        }
        public static Action Wristdown(){
            return new Wristdown();
        }
    }
}