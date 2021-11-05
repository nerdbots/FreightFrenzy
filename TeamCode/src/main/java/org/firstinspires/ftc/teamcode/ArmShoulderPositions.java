package org.firstinspires.ftc.teamcode;

public  enum ArmShoulderPositions {

    INTAKE(0,0.2,0,1.0),
    LEVEL1(1100,0.6,0.5,0.5),
    LEVEL2(860,0.6,1,0),
    LEVEL3(600,0.6,1,0),
    TSE_DROP(635,0.6,1,0),
    GROUND_PICKUP(1000,0.6,0.7,0.3);


    private final int armTarget;
    private final double maxPower;
    private final double leftWristServoPosition;
    private final double rightWristServoPosition;


    private ArmShoulderPositions(int motorPosition, double maxPower, double leftWristServoPosition, double rightWristServoPosition) {
        this.armTarget = motorPosition;
        this.maxPower = maxPower;
        this.leftWristServoPosition = leftWristServoPosition;
        this.rightWristServoPosition = rightWristServoPosition;

    }

    public int getArmTarget() {
        return this.armTarget;
    }

    public double getMaxPower() {
        return this.maxPower;
    }

    public double getLeftWristServoPosition() {
        return this.leftWristServoPosition;
    }
    public double getRightWristServoPosition() {
        return this.rightWristServoPosition;
    }

}