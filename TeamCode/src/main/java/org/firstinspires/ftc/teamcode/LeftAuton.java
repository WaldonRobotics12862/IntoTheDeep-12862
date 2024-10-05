package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.DiveActions;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Config
@Autonomous(name="LeftAuton", group="Autonomous")

public class LeftAuton extends LinearOpMode {

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
            return new LiftDown();
        }
    }

    public class Intake {
        private CRServo intake;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(CRServo.class, "intake");
        }

        public class RunIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(1.0);
                return false;
            }
        }
        public Action RunIntake() {
            return new RunIntake();
        }
        public class StopIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(0.0);
                return false;
            }
        }
        public Action StopIntake() {
            return new StopIntake();
        }


        public class RevIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(-1.0);
                return false;
            }
        }
        public Action RevIntake() {
            return new RevIntake();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-36, -63, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        Lift Lift = new Lift(hardwareMap);
        Intake Intake = new Intake(hardwareMap);

        Action AutonLeft = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(-6.25, -45.00), Math.toRadians(90.00))
                .waitSeconds(3)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-6.25, -50), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(-55.25, -50), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(-55.25, -46.0), Math.toRadians(90.00))
                .splineTo(new Vector2d(-58.5, -58), Math.toRadians(225))
                .splineTo(new Vector2d(-59, -47), Math.toRadians(90))
                .turn(Math.toRadians(215))
                .splineTo(new Vector2d(-67, -47), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-67, -14), Math.toRadians(90))
                .splineTo(new Vector2d(-20, -14), Math.toRadians(180.00))
                .build();

        //actions that need to happen on init (if any)



        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        AutonLeft,
                        Lift.LiftUp(),
                        DiveActions.sampleDelivery.dump(),
                        Intake.RunIntake()
                )
        );
    }
}