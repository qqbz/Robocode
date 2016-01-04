package git;

import java.awt.Color;
import java.util.Hashtable;
import java.awt.geom.Point2D;
import robocode.*;
import robocode.util.Utils;
import robocode.MessageEvent;
import robocode.TeamRobot;
import robocode.DeathEvent;
import static robocode.util.Utils.normalRelativeAngleDegrees;
//kommunikation mit 
public class SteinbeißerDroid extends TeamRobot implements Droid {

    public Hashtable<String, RobotInfo> robotsList = new Hashtable<String, RobotInfo>();
    RobotInfo robotInformation = new RobotInfo();
    String droidName;
    String leaderName;
    String target;
    public boolean movingForward = true;
    public boolean hanSoloMode = false;
    double firePower;

    @Override
    public void run() {

        RobotColors c = new RobotColors();
        c.bodyColor = Color.black;
        c.gunColor = Color.green;
        c.bulletColor = Color.YELLOW;
        setBodyColor(c.bodyColor);
        setGunColor(c.gunColor);
        setBulletColor(c.bulletColor);

        setEventPriority("RobotDeathEvent", 81);
        this.droidName = getName();
        
        do {
            if ((hanSoloMode == true)) {
                soloMode();

            }
            if (target != null) {
                goTo(robotsList.get(target));
                shoot();
                System.out.println(" got target");
            }
           
            execute();
       
        } while (true);

    }

    @Override
    public void onMessageReceived(MessageEvent e) {

        this.robotsList = (Hashtable<String, RobotInfo>) e.getMessage();
        this.leaderName = e.getSender();
        this.robotInformation = robotsList.get(leaderName);
        this.target = robotInformation.getTARGET();
        
    }
//schussrichtung zu ungenau muss noch besser werden
    public void shoot() {

        double dx = robotsList.get(target).getX() - this.getX();
        double dy = robotsList.get(target).getY() - this.getY();
        
        double distance = Math.hypot(dx, dy);
        double theta = Math.toDegrees(Math.atan2(dx, dy));

        turnGunRight(normalRelativeAngleDegrees(theta - getGunHeading()));

        System.out.println(" Target " + target + " Distance " + distance);

        if (distance >= 200) {
            this.firePower=0.8;
            System.out.println(" Fire with "+firePower);
        }
        if (distance > 50 && distance < 200) {
            this.firePower= 1.5;
            System.out.println(" Fire with " +firePower);
        }
        if (distance <= 50) {
            this.firePower= 3;
            System.out.println(" Fire with "+firePower);
        }

        fire(firePower);

    }

//    public double normalRelativeAngle(double angle) {
//        if (angle > -180 && angle <= 180) {
//            return angle;
//        }
//        double fixedAngle = angle;
//
//        while (fixedAngle <= -180) {
//            fixedAngle += 360;
//        }
//        while (fixedAngle > 180) {
//            fixedAngle -= 360;
//        }
//        return fixedAngle;
//    }
    
 //http://www.jasonsjava.com/?cat=2
    private void goTo(Point2D destination) {
//      stop(); // don't move again until we are ready
        Point2D location = new Point2D.Double(getX(), getY());
//        System.out.println("JETZIGE POSITION " + location);
        double distance = location.distance(destination);
        // this angle is the amount I need to turn right to face the destination point
        double angle = Utils.normalRelativeAngleDegrees(Math.toDegrees(Math.atan2(destination.getX() - location.getX(), destination.getY() - location.getY())) - getHeading());
        // this statement takes the previous angle and basically divides it in half.
        // if my rear end is closer, then turn the rear towards the destination and drive backwards.
        if (Math.abs(angle) > 90) {
            distance *= -1;  // drive backwards
            if (angle > 0) {
                angle -= 180;
            } else {
                angle += 180;
            }
        }

        turnRight(angle); // complete the turn before going any distance (blocking call from Robot class)
        setAhead(distance);
        execute();
        // must be called because setAhead() will not move without a call to execute().
    }


    @Override
    public void onSkippedTurn(SkippedTurnEvent e) {
        System.out.println("Round " + e.getSkippedTurn() + " was skipped!");
    }

    @Override
    public void onHitRobot(HitRobotEvent e) {
        if (isTeammate(e.getName())) {
            reverseDirection();
//            turnLeft(e.getBearing() + 90);
//            ahead(100);
//            System.out.println("I have hit the Leader");
        }


    }

    @Override
    public void onHitWall(HitWallEvent e) {
        out.println("I hit the wall " + e.getBearing());
        reverseDirection();
//        turnLeft(event.getBearing() + 45);
//        ahead(120);
    }

//wird vlt. noch gebraucht
    @Override
    public void onDeath(DeathEvent e) {
    }

    @Override
    public void onRobotDeath(RobotDeathEvent e) {

        if (isTeammate(e.getName())) {
            System.out.println(e.getName() + " is Dead!");
            hanSoloMode = true;
        }

    }
    
//aus sample.crazy bot
    public void soloMode() {
        setAhead(40000);
        movingForward = true;
        setTurnRight(90);
        waitFor(new TurnCompleteCondition(this));
        setTurnLeft(180);
        waitFor(new TurnCompleteCondition(this));
        setTurnRight(180);
        waitFor(new TurnCompleteCondition(this));

    }

    public void reverseDirection() {
        if (movingForward) {
            setMaxVelocity(6);
            setMaxTurnRate(Rules.MAX_TURN_RATE);
            setBack(100);
            movingForward = false;
        } else {
            setMaxVelocity(6);
            setMaxTurnRate(Rules.MAX_TURN_RATE);
            setAhead(100);
            movingForward = true;
        }
    }
}
