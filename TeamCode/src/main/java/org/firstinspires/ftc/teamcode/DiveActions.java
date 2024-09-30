package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DiveActions {
    public class Lift {
        private DcMotorEx liftLeft;
        private DcMotorEx liftRight;

        public Lift(HardwareMap hardwareMap) {
            liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeftMotor");
            liftLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftLeft.setDirection(DcMotorEx.Direction.FORWARD);

            liftRight = hardwareMap.get(DcMotorEx.class, "liftRightMotor");
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
                public Action Open(){ return new Open;}

                    }
            }


        public Action LiftUp() {
            return new LeftAuton.Lift.LiftUp();
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
                if (posR >10.0 & posL > 10.0) {
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
            return new LeftAuton.Lift.LiftDown();
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
