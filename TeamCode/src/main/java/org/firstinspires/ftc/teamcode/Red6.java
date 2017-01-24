//PRIMARY AUTONOMOUS PROGRAM
//TwoShootCenterRed



package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**

  This works

 */
//LinearOpMode
    //started 12/19 by Justin

@Autonomous(name = "Red6", group = "Auto")

public class Red6 extends LinearOpMode {

    HardwarePushbotTDR robot = new HardwarePushbotTDR();
    VuforiaOp camera = new VuforiaOp();
    private ElapsedTime runtime = new ElapsedTime();
    Boolean beaconOneRed;
    Boolean beaconTwoRed;
    Boolean doneDrive1 = false;
    Boolean beaconOneDone = false;
    Boolean beaconTwoDone = false;
    Boolean forwardTwoDone = false;
    Boolean turnTwoDone = false;
    Boolean longDriveDone = false;
    Boolean followOneDone = false;
    Boolean beaconOneWrong = false;
    Boolean beaconTwoWrong = false;

    Boolean shoot = false;


    double vr = 1;//change for direction and battery
    double vl = 1;//change for direction and battery
    double shotSpeed = .39;//change for battery
    int step = 0;
    double shot = 0;
    double lastPosL = 0;
    double lastPosR = 0;
    double twolastPosL = 0;
    double lastClock = 0;
    int beaconOneCount = 0;
    int BeaconTwoCount = 0;
    int forwardTwoCount = 0;
    int turnTwoCount = 0;
    int followOneCount = 0;
    int turnOneCount = 0;
    double startPosR = 0;
    String status = "Start";


    /*
        public OneShootCenterRed(){

        }
    */
    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);

        double startPosL = robot.MotorL.getCurrentPosition();
        robot.LiftServo.setPosition(.88);
        robot.ShotFeeder.setPosition(.9);
        robot.CapGateServo.setPosition(1);//in
        robot.PressServoR.setPosition(1);//in
        robot.PressServoL.setPosition(0);//in
        robot.TouchServo.setPosition(.1);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.CapGateServo.setPosition(.5);

        //shoot first ball then move servo so it can help hold up the ball..

        status = "Start, move servo";
        telemetry.addData("Status:", status);
        telemetry.update();

        startPosL = robot.MotorL.getCurrentPosition();
        status = "drive back until white line";
        shot = 0;
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() > startPosL - 4350 && !doneDrive1) {//stop if hit line or go certain distance
            if (robot.ColSensor.blue() > 20) {
                doneDrive1 = true;//hit white line
            }
            robot.MotorL.setPower(-.85 * vl);
            robot.MotorR.setPower(-.85 * vr);
            robot.PressServoL.setPosition(0);//in
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL current", robot.MotorL.getCurrentPosition() - startPosL);
            telemetry.addData("sensorColor:", robot.ColSensor.blue());
            telemetry.update();

            if (shoot) {
                //turn on shooter
                if (shot < shotSpeed) {
                    shot += 0.02;
                    robot.ShooterDown.setPower(shot);
                    robot.ShooterUp.setPower(-shot);
                } else {
                    robot.ShooterDown.setPower(shotSpeed);
                    robot.ShooterUp.setPower(-shotSpeed);
                }
            }
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "drive back past line";
        startPosL = robot.MotorL.getCurrentPosition();
        robot.MotorL.setPower(-.35 * vl);
        robot.MotorR.setPower(-.35 * vr);
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() > startPosL - 950) && doneDrive1) {//only does this step if hit white line
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL units to go", robot.MotorL.getCurrentPosition() - startPosL + 950);
            telemetry.addData("MotorL current", robot.MotorL.getCurrentPosition());
            telemetry.update();
            idle();

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "turn until white line";
        robot.TouchServo.setPosition(1);//out
        turnOneCount = 0;
        lastPosL = robot.MotorL.getCurrentPosition();
        lastPosR = robot.MotorR.getCurrentPosition();
        lastClock = runtime.seconds();
        while (opModeIsActive() && robot.ColSensor.blue() < 20) {
            telemetry.addData("change in time", runtime.seconds() - lastClock);
            telemetry.addData("Status:", status);
            telemetry.update();
            robot.MotorL.setPower(-.41 * vl);
            robot.MotorR.setPower(.41 * vr);

            /*if(runtime.seconds() - lastClock > 1.7){
                lastPosL = robot.MotorL.getCurrentPosition();
                lastPosR = robot.MotorR.getCurrentPosition();
                lastClock = runtime.seconds();
                if((Math.abs(lastPosL - robot.MotorL.getCurrentPosition()) < 200)
                        && (Math.abs(lastPosR - robot.MotorR.getCurrentPosition()) < 200)){
                    while(robot.MotorL.getCurrentPosition() < lastPosL + 100){
                        robot.MotorL.setPower(.33 *vl);
                        robot.MotorR.setPower(.33 * vr);
                    }
                    robot.MotorL.setPower(0);
                    robot.MotorR.setPower(0);
                }
            }*/
            telemetry.addData("lastPosL", lastPosL);
            telemetry.addData("current", robot.MotorL.getCurrentPosition());
            telemetry.addData("turnOneCount", turnOneCount);

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "wait";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.2) {
            robot.MotorL.setPower(0);
            robot.MotorR.setPower(0);
            telemetry.addData("Status:", status);
            telemetry.update();

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "turn a bit";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosL + 110) {
            robot.MotorL.setPower(.3 * vl);
            robot.MotorR.setPower(-.3 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL - 110);
            telemetry.update();

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "line follow";
        startPosL = robot.MotorL.getCurrentPosition();
        lastPosL = robot.MotorL.getCurrentPosition();
        lastPosR = robot.MotorR.getCurrentPosition();
        lastClock = runtime.seconds();
        twolastPosL = 0;
        while (opModeIsActive() && !robot.TouSensor.isPressed()) {
            telemetry.addData("Status:", status);
            telemetry.addData("TouSensor.isPressed()", robot.TouSensor.isPressed());
            telemetry.update();
            if (robot.ColSensor.blue() < 20) {//grey
                robot.MotorR.setPower(.2 * vl);
                robot.MotorL.setPower(-.4 * vr);
            } else if (robot.ColSensor.blue() > 20) {//white
                robot.MotorR.setPower(-.4 * vl);
                robot.MotorL.setPower(.2 * vr);
            }

            /*if(runtime.seconds() - lastClock > 2){
                lastPosL = robot.MotorL.getCurrentPosition();
                lastPosR = robot.MotorR.getCurrentPosition();
                lastClock = runtime.seconds();
                if((Math.abs(lastPosL - robot.MotorL.getCurrentPosition()) < 400)//if neither motor has changed by more than 200, back up
                        && (Math.abs(lastPosR - robot.MotorR.getCurrentPosition()) < 400)){
                    while(robot.MotorL.getCurrentPosition() < lastPosL + 100){
                        robot.MotorL.setPower(.33 *vl);
                        robot.MotorR.setPower(.33 * vr);
                    }
                    robot.MotorL.setPower(0);
                    robot.MotorR.setPower(0);
                }
            }*/
            telemetry.addData("Time", runtime.seconds());
            telemetry.addData(" Change in last and current", Math.abs(lastPosL - robot.MotorL.getCurrentPosition()));
            telemetry.addData("followOneCount", followOneCount);

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "wait";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.3) {
            robot.MotorL.setPower(0);
            robot.MotorR.setPower(0);
            telemetry.addData("Status:", status);
            telemetry.update();

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "turn a bit";
        telemetry.update();
        startPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 70) {//motorR is reversed
            robot.MotorL.setPower(-.38 * vl);
            robot.MotorR.setPower(.38 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorR to go", robot.MotorR.getCurrentPosition() - startPosR + 70);
            telemetry.update();

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "sense color";
        telemetry.update();
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);
        startPosL = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .1) {
            telemetry.addData("Status:", status);
            telemetry.addData("sensorColor:", robot.ColSensor.blue());
            telemetry.update();
            if (robot.FruitySensor.blue() > robot.FruitySensor.red()) {
                beaconOneRed = false;
            } else {
                beaconOneRed = true;
            }

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "forward, touch servo in";
        startPosL = robot.MotorL.getCurrentPosition();
        lastClock = runtime.seconds();
        robot.TouchServo.setPosition(.1);//in
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() < startPosL + 200)) {
            robot.MotorL.setPower(.3 * vl);
            robot.MotorR.setPower(.3 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("currentPos - startPosL + 400", robot.MotorL.getCurrentPosition() - startPosL - 200);
            telemetry.update();

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }

        status = "turn paddles";
        telemetry.update();
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);
        startPosL = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .5) {
            telemetry.addData("Status:", status);
            telemetry.update();
            if (beaconOneRed) {
                robot.PressServoR.setPosition(0);//right out
                robot.PressServoL.setPosition(0);//left in
            } else {
                robot.PressServoR.setPosition(1);//right in
                robot.PressServoL.setPosition(1);//left out
            }

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }

        status = "backward and press buttons, beacon 1";
        startPosL = robot.MotorL.getCurrentPosition();
        lastPosL = robot.MotorL.getCurrentPosition();
        lastPosR = robot.MotorR.getCurrentPosition();
        lastClock = runtime.seconds();
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() > startPosL - 350) && !beaconOneDone) {
            robot.MotorL.setPower(-.5 * vl);
            robot.MotorR.setPower(-.5 * vr);

            telemetry.addData("Status:", status);
            telemetry.addData("currentPos - startPosL + 400", robot.MotorL.getCurrentPosition() - startPosL + 300);
            telemetry.addData("beaconOneCount", beaconOneCount);
            telemetry.addData("beaconOneDone", beaconOneDone);
            telemetry.update();

            if (runtime.seconds() - lastClock > 1.7) {
                lastPosL = robot.MotorL.getCurrentPosition();
                lastPosR = robot.MotorR.getCurrentPosition();
                lastClock = runtime.seconds();
                if ((Math.abs(lastPosL - robot.MotorL.getCurrentPosition()) < 90)
                        && (Math.abs(lastPosR - robot.MotorR.getCurrentPosition()) < 90)) {
                    beaconOneDone = true;
                }
            }

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "check correct color";
        lastClock = runtime.seconds();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1) {//.1 1/22
            telemetry.addData("blue-red", robot.FruitySensor.blue() - robot.FruitySensor.red());
            telemetry.addData("blue", robot.FruitySensor.blue());
            telemetry.addData("red", robot.FruitySensor.red());
            telemetry.update();

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        if (robot.FruitySensor.blue() > robot.FruitySensor.red()) {
            beaconOneWrong = true;
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "forward if wrong color";
        startPosL = robot.MotorL.getCurrentPosition();
        lastClock = runtime.seconds();
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() < startPosL + 100) && beaconOneWrong) {
            robot.MotorL.setPower(.3 * vl);
            robot.MotorR.setPower(.3 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("currentPos - startPosL + 400", robot.MotorL.getCurrentPosition() - startPosL - 100);
            telemetry.addData("beaconOneCount", beaconOneCount);
            telemetry.addData("beaconOneDone", beaconOneDone);
            telemetry.update();

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "wait if wrong color";
        robot.PressServoL.setPosition(1);
        robot.PressServoR.setPosition(0);//both paddles out
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5.2) && beaconOneWrong) {
            robot.MotorL.setPower(0);
            robot.MotorR.setPower(0);
            telemetry.addData("Status:", status);
            telemetry.update();

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "backward and press buttons, beacon 1, second try";
        startPosL = robot.MotorL.getCurrentPosition();
        lastPosL = robot.MotorL.getCurrentPosition();
        lastPosR = robot.MotorR.getCurrentPosition();
        lastClock = runtime.seconds();
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() > startPosL - 200) && beaconOneWrong) {
            robot.MotorL.setPower(-.3 * vl);
            robot.MotorR.setPower(-.3 * vr);

            telemetry.addData("Status:", status);
            telemetry.addData("currentPos - startPosL + 400", robot.MotorL.getCurrentPosition() - startPosL + 200);
            telemetry.addData("beaconOneCount", beaconOneCount);
            telemetry.addData("beaconOneDone", beaconOneDone);
            telemetry.update();

            if (runtime.seconds() - lastClock > 1.7) {
                lastPosL = robot.MotorL.getCurrentPosition();
                lastPosR = robot.MotorR.getCurrentPosition();
                lastClock = runtime.seconds();
                if ((Math.abs(lastPosL - robot.MotorL.getCurrentPosition()) < 90)
                        && (Math.abs(lastPosR - robot.MotorR.getCurrentPosition()) < 90)) {
                    beaconOneDone = true;
                }
            }

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);


        status = "forward off beacon 1";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosL + 850) {
            robot.MotorL.setPower(.6 * vl);
            robot.MotorR.setPower(.6 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL - 850);
            telemetry.update();

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "wait";
        telemetry.update();
        robot.PressServoL.setPosition(0);
        robot.PressServoR.setPosition(1);
        startPosL = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.5) {
            robot.MotorL.setPower(0);
            robot.MotorR.setPower(0);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL + 1250);
            telemetry.update();

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        if (shoot) {
            status = "shoot first ball";
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1.5) {
                shot = shotSpeed;
                robot.ShotFeeder.setPosition(0);//shoot
                robot.ShooterDown.setPower(shot);
                robot.ShooterUp.setPower(-shot);
                telemetry.addData("shot", shot);
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.addData("Status:", status);
                telemetry.update();
            }

            status = "feed second ball";
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 2) {
                robot.ShotFeeder.setPosition(.9);//down
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.addData("Status:", status);
                telemetry.update();
            }

            status = "shoot second ball";
            robot.Conveyor.setPower(0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1.5) {
                telemetry.addData("shot", shot);
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.addData("Status:", status);
                telemetry.update();
                    shot = shotSpeed;
                    robot.ShotFeeder.setPosition(0);//shoot
                    robot.ShooterDown.setPower(shotSpeed);
                    robot.ShooterUp.setPower(-shotSpeed);
            }
            robot.PressServoL.setPosition(0);//left in
            robot.PressServoR.setPosition(1);//left in
        }

        status = "forward after shoot";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosL + 1000) {
            robot.MotorL.setPower(.6 * vl);
            robot.MotorR.setPower(.6 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL - 1000);
            telemetry.update();

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "turn to beacon 2";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosL + 1050) {
            robot.MotorL.setPower(.5 * vl);
            robot.MotorR.setPower(-.5 * vr);
            robot.PressServoR.setPosition(1);//in
            robot.PressServoL.setPosition(0);//in
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL + 1050);
            telemetry.update();

            if (shoot) {
                robot.ShooterDown.setPower(shotSpeed);
                robot.ShooterUp.setPower(-shotSpeed);
            }
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "drive until white line";
        robot.TouchServo.setPosition(1);
        startPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() > startPosL - 3700) && !longDriveDone) {//stop if hit line or go certain distance
            robot.MotorL.setPower(-.85 * vl);
            robot.MotorR.setPower(-.85 * vr);
            if (robot.ColSensor.blue() > 20) {
                longDriveDone = true;//hit white line
            }
            telemetry.addData("Status:", status);
            telemetry.addData("sensorColor:", robot.ColSensor.blue());
            telemetry.update();

            if (shoot) {
                if (shot > 0.18) {
                    shot -= 0.02;
                    robot.ShotFeeder.setPosition(.9);
                    robot.ShooterDown.setPower(shot);
                    robot.ShooterUp.setPower(-shot);
                } else if (shot < 0.18 && shot > 0) {
                    shot -= 0.0025;
                    robot.ShooterDown.setPower(shot);
                    robot.ShooterUp.setPower(-shot);
                } else {
                    robot.ShooterDown.setPower(0);
                    robot.ShooterUp.setPower(0);
                }
            }

            telemetry.addData("Shot:", shot);
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);
        robot.ShooterDown.setPower(0);
        robot.ShooterUp.setPower(0);

        status = "backward past line";
        startPosL = robot.MotorL.getCurrentPosition();
        robot.MotorL.setPower(-.3 * vl);
        robot.MotorR.setPower(-.3 * vr);
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() > startPosL - 550) && longDriveDone) {//stop if go certain distance or stuck on wall
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL units to go", robot.MotorL.getCurrentPosition() - startPosL + 550);
            telemetry.addData("MotorL current", robot.MotorL.getCurrentPosition());
            telemetry.addData("forwardTwoCount", forwardTwoCount);
            telemetry.addData("forwardTwoDone", forwardTwoDone);
            telemetry.update();
            lastPosL = robot.MotorL.getCurrentPosition();
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);


        status = "turn until white line, beacon 2";
        turnOneCount = 0;
        lastPosL = robot.MotorL.getCurrentPosition();
        lastPosR = robot.MotorR.getCurrentPosition();
        lastClock = runtime.seconds();
        while (opModeIsActive() && robot.ColSensor.blue() < 20) {
            telemetry.addData("change in time", runtime.seconds() - lastClock);
            telemetry.addData("Status:", status);
            telemetry.update();
            robot.MotorL.setPower(-.41 * vl);
            robot.MotorR.setPower(.41 * vr);

            if (runtime.seconds() - lastClock > 2.7) {
                lastPosL = robot.MotorL.getCurrentPosition();
                lastPosR = robot.MotorR.getCurrentPosition();
                lastClock = runtime.seconds();
                if ((Math.abs(lastPosL - robot.MotorL.getCurrentPosition()) < 200)
                        && (Math.abs(lastPosR - robot.MotorR.getCurrentPosition()) < 200)) {
                    while (robot.MotorL.getCurrentPosition() < lastPosL + 100) {
                        robot.MotorL.setPower(.33 * vl);
                        robot.MotorR.setPower(.33 * vr);
                    }
                    robot.MotorL.setPower(0);
                    robot.MotorR.setPower(0);
                }
            }
            telemetry.addData("lastPosL", lastPosL);
            telemetry.addData("current", robot.MotorL.getCurrentPosition());
            telemetry.addData("turnOneCount", turnOneCount);
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "wait";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.2) {
            robot.MotorL.setPower(0);
            robot.MotorR.setPower(0);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL + 1250);
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "turn a bit";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosL + 110) {
            robot.MotorL.setPower(.3 * vl);
            robot.MotorR.setPower(-.3 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL + 1250);
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "line follow, beacon 2";
        startPosL = robot.MotorL.getCurrentPosition();
        lastPosL = robot.MotorL.getCurrentPosition();
        lastPosR = robot.MotorR.getCurrentPosition();
        lastClock = runtime.seconds();
        while (opModeIsActive() && !robot.TouSensor.isPressed()) {
            telemetry.addData("Status:", status);
            telemetry.addData("TouSensor.isPressed()", robot.TouSensor.isPressed());
            telemetry.update();
            if (robot.ColSensor.blue() < 20) {//grey
                robot.MotorR.setPower(.2 * vl);
                robot.MotorL.setPower(-.4 * vr);
            } else if (robot.ColSensor.blue() > 20) {//white
                robot.MotorR.setPower(-.4 * vl);
                robot.MotorL.setPower(.2 * vr);
            }

            /*if(runtime.seconds() - lastClock > 2){
                lastPosL = robot.MotorL.getCurrentPosition();
                lastPosR = robot.MotorR.getCurrentPosition();
                lastClock = runtime.seconds();
                if((Math.abs(lastPosL - robot.MotorL.getCurrentPosition()) < 200)
                        && (Math.abs(lastPosR - robot.MotorR.getCurrentPosition()) < 200)){
                    while(robot.MotorL.getCurrentPosition() < lastPosL + 100){
                        robot.MotorL.setPower(.33 *vl);
                        robot.MotorR.setPower(.33 * vr);
                    }
                    robot.MotorL.setPower(0);
                    robot.MotorR.setPower(0);
                }
            }*/
            telemetry.addData("Time", runtime.seconds());
            telemetry.addData(" Change in last and current", Math.abs(lastPosL - robot.MotorL.getCurrentPosition()));
            telemetry.addData("followOneCount", followOneCount);
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "wait";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.3) {
            robot.MotorL.setPower(0);
            robot.MotorR.setPower(0);
            telemetry.addData("Status:", status);
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "turn a bit";
        telemetry.update();
        startPosR = robot.MotorR.getCurrentPosition();
        while (opModeIsActive() && robot.MotorR.getCurrentPosition() < startPosR + 70) {
            robot.MotorL.setPower(-.35 * vl);
            robot.MotorR.setPower(.35 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorR to go", robot.MotorR.getCurrentPosition() - startPosL - 70);
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "sense color, beacon 2";
        telemetry.update();
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);
        startPosL = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .1) {
            telemetry.addData("Status:", status);
            telemetry.addData("sensorColor:", robot.ColSensor.blue());
            telemetry.update();
            if (robot.FruitySensor.blue() > robot.FruitySensor.red()) {
                beaconOneRed = false;
            } else {
                beaconOneRed = true;
            }
        }

        status = "forward for touch servo";
        startPosL = robot.MotorL.getCurrentPosition();
        lastClock = runtime.seconds();
        robot.TouchServo.setPosition(.1);
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() < startPosL + 100)) {
            robot.MotorL.setPower(.3 * vl);
            robot.MotorR.setPower(.3 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("currentPos - startPosL + 400", robot.MotorL.getCurrentPosition() - startPosL - 100);
            telemetry.addData("beaconOneCount", beaconOneCount);
            telemetry.addData("beaconOneDone", beaconOneDone);
            telemetry.update();
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "turn paddles, beacon 2";
        telemetry.update();
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);
        startPosL = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .7) {
            telemetry.addData("Status:", status);
            telemetry.update();
            if (beaconOneRed) {
                robot.PressServoR.setPosition(0);//out
            } else {
                robot.PressServoL.setPosition(1);//in
            }
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "backward and press buttons, beacon 2";
        startPosL = robot.MotorL.getCurrentPosition();
        lastPosL = robot.MotorL.getCurrentPosition();
        lastPosR = robot.MotorR.getCurrentPosition();
        lastClock = runtime.seconds();
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() > startPosL - 300) && !beaconTwoDone) {
            robot.MotorL.setPower(-.5 * vl);
            robot.MotorR.setPower(-.5 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("currentPos - startPosL + 400", robot.MotorL.getCurrentPosition() - startPosL + 200);
            telemetry.addData("beaconOneCount", beaconOneCount);
            telemetry.addData("beaconTwoDone", beaconTwoDone);
            telemetry.update();

            if (runtime.seconds() - lastClock > 1.7) {
                lastPosL = robot.MotorL.getCurrentPosition();
                lastPosR = robot.MotorR.getCurrentPosition();
                lastClock = runtime.seconds();
                if ((Math.abs(lastPosL - robot.MotorL.getCurrentPosition()) < 90)
                        && (Math.abs(lastPosR - robot.MotorR.getCurrentPosition()) < 90)) {
                    beaconTwoDone = true;
                }
            }
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "check correct color";
        lastClock = runtime.seconds();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1) {//.1 1/22
            telemetry.addData("blue-red", robot.FruitySensor.blue() - robot.FruitySensor.red());
            telemetry.addData("blue", robot.FruitySensor.blue());
            telemetry.addData("red", robot.FruitySensor.red());
            telemetry.update();
        }
        if (robot.FruitySensor.blue() > robot.FruitySensor.red()) {
            beaconTwoWrong = true;
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "forward if wrong color";
        startPosL = robot.MotorL.getCurrentPosition();
        lastClock = runtime.seconds();
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() < startPosL + 100) && beaconTwoWrong) {
            robot.MotorL.setPower(.3 * vl);
            robot.MotorR.setPower(.3 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("currentPos - startPosL + 400", robot.MotorL.getCurrentPosition() - startPosL - 100);
            telemetry.addData("beaconOneCount", beaconOneCount);
            telemetry.addData("beaconOneDone", beaconOneDone);
            telemetry.update();
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "wait if wrong color";
        robot.PressServoL.setPosition(1);
        robot.PressServoR.setPosition(0);
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5.2) && beaconTwoWrong) {
            robot.MotorL.setPower(0);
            robot.MotorR.setPower(0);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL + 1250);
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "backward and press buttons, beacon 1, second try";
        startPosL = robot.MotorL.getCurrentPosition();
        lastPosL = robot.MotorL.getCurrentPosition();
        lastPosR = robot.MotorR.getCurrentPosition();
        lastClock = runtime.seconds();
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() > startPosL - 200) && beaconTwoWrong) {
            robot.MotorL.setPower(-.3 * vl);
            robot.MotorR.setPower(-.3 * vr);

            telemetry.addData("Status:", status);
            telemetry.addData("currentPos - startPosL + 400", robot.MotorL.getCurrentPosition() - startPosL + 200);
            telemetry.addData("beaconOneCount", beaconOneCount);
            telemetry.addData("beaconOneDone", beaconOneDone);
            telemetry.update();

            if (runtime.seconds() - lastClock > 1.7) {
                lastPosL = robot.MotorL.getCurrentPosition();
                lastPosR = robot.MotorR.getCurrentPosition();
                lastClock = runtime.seconds();
                if ((Math.abs(lastPosL - robot.MotorL.getCurrentPosition()) < 90)
                        && (Math.abs(lastPosR - robot.MotorR.getCurrentPosition()) < 90)) {
                    beaconOneDone = true;
                }
            }
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "forward off beacon 2";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosL + 1000) {
            robot.MotorL.setPower(.4 * vl);
            robot.MotorR.setPower(.4 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL - 1000);
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "turn to center";
        telemetry.update();
        robot.PressServoL.setPosition(0);
        robot.PressServoR.setPosition(1);//both in
        startPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosL + 800) {
            robot.MotorL.setPower(.6 * vl);
            robot.MotorR.setPower(-.6 * vr);
            robot.PressServoR.setPosition(1);//in
            robot.PressServoL.setPosition(0);//out
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL - 00);
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "drive and park";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosL + 4500) {
            robot.MotorL.setPower(.8 * vl);
            robot.MotorR.setPower(.8 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL - 4500);
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        telemetry.addData("Shot:", shot);
        if (shot > 0) {
            while (shot > 0) {
                if (shot > 0.18) {
                    shot -= 0.02;

                } else if (shot < 0.18) {
                    shot -= 0.0025;
                }
                robot.ShooterDown.setPower(shot);
                robot.ShooterUp.setPower(-shot);
            }
            robot.ShooterDown.setPower(0);
            robot.ShooterUp.setPower(0);


        }


    }

}



