package git;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.LinkedHashMap;
import java.util.logging.Level;
import java.util.logging.Logger;
import robocode.ScannedRobotEvent;
import robocode.TeamRobot;
import robocode.MessageEvent;
import robocode.RobotDeathEvent;
import robocode.SkippedTurnEvent;
import robocode.util.Utils;

/**
 * SteinbeißerLeader mit Radar
 *
 * @author Lukas Becker, Alex Dercho, Simon Fella
 */
public class SteinbeißerLeader extends TeamRobot {

    /**
     * Die Liste aller gescannten Roboter und des Leaders mit deren Daten.
     */
    Hashtable<String, RobotInfo> robots = new Hashtable<>();
    /**
     * Das aktuelle Ziel.
     */
    String target;
    /**
     * Boolean Variable für geändertes Verhalten wenn der Droid zerstört wurde.
     */
    boolean soloMode = false;
    /**
     * Boolean Variable für das radar-lock bei einem einzelnen Gegner
     */
    boolean singleEnemy = false;
    /**
     *
     */
    Point2D goal;
    /**
     *
     */
    int targetingTime;
    /**
     *
     */
    double xForce, yForce;
    /**
     *
     */
    double angle;
    /**
     *
     */
    Point2D destination;
    static double scanDir;
    static Object sought;
    Rectangle2D rec;
    static LinkedHashMap<String, Double> enemyHashMap;
    private double oldEnemyHeading;
    double a,b;

    /*
     TO DO:
     chooseTarget(): schnelles Ändern der Target verhindern 
     getFirepower(): Grenzwerte mit Formel festlegen
     Probleme bestehen beim gerammt werden: wenn ein Bot im Weg steht zum idealen Punkt und alle neuen idealen Punkte auch hinter diesem Bot liegen
            hängt der Leader die ganze Zeit am Bot und fährt nicht weg
     */
    @Override
    public void run() {
        RobotColors c = new RobotColors(); //setzt alle Farben
        c.bodyColor = Color.black;
        c.gunColor = Color.green;
        c.radarColor = Color.green;
        c.scanColor = Color.green;
        c.bulletColor = Color.green;
        setBodyColor(c.bodyColor);
        setGunColor(c.gunColor);
        setRadarColor(c.radarColor);
        setScanColor(c.scanColor);
        setBulletColor(c.bulletColor); //

        setAdjustRadarForGunTurn(true); //für Radar notwendig
        setAdjustGunForRobotTurn(true); 
        setAdjustRadarForRobotTurn(true);//
        scanDir = 1;
        enemyHashMap = new LinkedHashMap<String, Double>(5, 2, true);

        RobotInfo leader = new RobotInfo(); //fügt sich selbst der Liste hinzu
        robots.put(this.getName(), leader);
        leader.setName(this.getName()); //

        setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
        
        do {
            leader = (RobotInfo) robots.get(this.getName()); //updated die eigenen Werte
            leader.setLocation(this.getX(), this.getY());
            leader.setEnergy(this.getEnergy());
            leader.setHeadingRad(this.getHeading());
            leader.setVelocity(this.getVelocity());
            leader.setHeadingRad(this.getHeadingRadians()); //
            
            doRadar();
            chooseTarget();
            aimAndShoot();

            try { //sendet Liste und das momentane Ziel an den Droid
                broadcastMessage(robots);
                broadcastMessage(target);
            } catch (IOException ex) {
                Logger.getLogger(SteinbeißerLeader.class.getName()).log(Level.SEVERE, null, ex);
            } //

            move();
            execute();

        } while (true);

    }

    /**
     * Graphisches Debuggen.
     *
     * @param g Graphics2D
     */
    @Override
    public void onPaint(Graphics2D g) {
        try {
            g.setColor(Color.red);
            g.fillRect((int) m.getX() - 10, (int) m.getY() - 10, 20, 20);
            g.setColor(Color.WHITE);
            g.drawLine((int)getX(), (int)getY(), (int)a, (int)b);
        } catch (Exception e) {

        }
    }
    
    /**
     * Schaltet in den soloMode wenn der Droid seinen Tod meldet oder updated
     * die Position des Droid.
     *
     * @param e MessageEvent
     */
    @Override
    public void onMessageReceived(MessageEvent e) { //schaltet in den soloMode wenn der Droid seinen Tod meldet oder updated die Position des Droid
        RobotInfo droid = (RobotInfo) robots.get(e.getSender());
        if (e.getMessage() == "I'm dead!") {
            soloMode = true;
            robots.remove(e.getSender());
        } else if (e.getMessage() instanceof Point2D) {
            droid.setLocation((Point2D) e.getMessage());
        }
    }

    /**
     * SpinningRadar wenn mehrere Gegner vorhanden sind, Radar-lock wenn nur ein
     * Gegner vorhanden ist.
     */
    private void doRadar() { //zunächst nur spinning radar
        if (singleEnemy){
                scan();
            } else {
                setTurnRadarRightRadians(scanDir * Double.POSITIVE_INFINITY);
                scan();
            }
            
            if (getOthers() == 1 && !singleEnemy){
                singleEnemy = true;
                System.out.println("single enemy");
            }

    }

    /**
     * Zielt und schießt. Circular Tergeting
     */
    private void aimAndShoot() { //http://robowiki.net/wiki/Circular_Targeting
        if (target != null) {
            RobotInfo e = robots.get(target);
            double bulletPower = getFirepower();//Math.min(3.0, getEnergy());
            double myX = getX();
            double myY = getY();
            double absoluteBearing = getHeadingRadians() + e.getBearingRad();
            double enemyX = getX() + e.distance(getX(), getY()) * Math.sin(absoluteBearing);
            double enemyY = getY() + e.distance(getX(), getY()) * Math.cos(absoluteBearing);
            double enemyHeading = e.getHeadingRad();
            double enemyHeadingChange = enemyHeading - oldEnemyHeading;
            double enemyVelocity = e.getVelocity();
            oldEnemyHeading = enemyHeading;

            double deltaTime = 0;
            double battleFieldHeight = getBattleFieldHeight(),
                    battleFieldWidth = getBattleFieldWidth();
            double predictedX = enemyX, predictedY = enemyY;
            while ((++deltaTime) * (20.0 - 3.0 * bulletPower)
                    < Point2D.Double.distance(myX, myY, predictedX, predictedY)) {
                predictedX += Math.sin(enemyHeading) * enemyVelocity;
                predictedY += Math.cos(enemyHeading) * enemyVelocity;
                enemyHeading += enemyHeadingChange;
                if (predictedX < 18.0
                        || predictedY < 18.0
                        || predictedX > battleFieldWidth - 18.0
                        || predictedY > battleFieldHeight - 18.0) {

                    predictedX = Math.min(Math.max(18.0, predictedX),
                            battleFieldWidth - 18.0);
                    predictedY = Math.min(Math.max(18.0, predictedY),
                            battleFieldHeight - 18.0);
                    break;
                }
            }
            double theta = Utils.normalAbsoluteAngle(Math.atan2(
                    predictedX - getX(), predictedY - getY()));
            a = getX() + 1500 * Math.sin(theta);
            b = getY() + 1500 * Math.cos(theta);
            theta = Utils.normalRelativeAngle(
                    theta - getGunHeadingRadians());
            setTurnGunRightRadians(theta);
            System.out.println(bulletPower);
            fire(bulletPower);
        }
    }

    /**
     * Berechnet die Feuerkraft in Abhängigkeit von der Entfernung zum
     * momentanen Ziel.
     *
     * @return Berechnete Feuerkraft
     */
    private double getFirepower() { //errechnet die beste Feuerkraft je nach Entfernung zur target
        //Grenzen sind momentan noch grobe Schätzungen
        //Formel: Schadeneffizienz = E =   (6x-2)*min(1, 18/(d*asin(8/(20-3x)))     /     (10+ceil(2*x)
        //wobei x = Firepower; d = distance
        double power;
        double distance;
        RobotInfo bot = (RobotInfo) robots.get(target);
        distance = bot.distance(this.getX(), this.getY());
        if (distance > 300) {
            power = 0.8;
        } else if (distance < 75) {
            power = robocode.Rules.MAX_BULLET_POWER;
        } else {
            power = 1.5;
        }
        return power;
    }

    Point2D m;

    /**
     * Berechnet und bewegt sich in die neue Bewegungsrichtung.
     */
    private void move() {
        if (this.getDistanceRemaining() == 0.) {
            m = evaluate(generate());
            goTo((int) m.getX(), (int) m.getY());
        }
    }

    /**
     * Lässt den Bot zu den angegebenen Koordinaten fahren. Kleine Abweichungen sind möglich.
     * @param x x-Koordinate des Zielpunktes
     * @param y y-Koordinate des Zielpunktes
     */
    private void goTo(int x, int y) { //zwei alternative Methoden, beide aus dem Wiki
        //http://robowiki.net/wiki/GoTo
        double alpha;
        setTurnRightRadians(Math.tan(
                alpha = Math.atan2(x -= (int) getX(), y -= (int) getY())
                - getHeadingRadians()));
        setAhead(Math.hypot(x, y) * Math.cos(alpha));
    }
    
    /**
     * Wählt den sichersten Punkt im übergebenen Array aus.
     * @param pointarray Array aller Punkte die evaluiert werden sollen
     * @return den sichersten Punkt
     */
    private Point2D evaluate(Point2D pointarray[]) {
        double minrisk = Double.POSITIVE_INFINITY;
        double thisrisk;
        double energyratio;
        double perpendicularity;
        double distanceSq;
        Point2D bestPoint = new Point2D.Double();
        for (Point2D p : pointarray) {
            if (p != null) {
                thisrisk = 0;
                for (String key : robots.keySet()) {
                    RobotInfo r = robots.get(key);
                    energyratio = r.getEnergy() / this.getEnergy();
                    perpendicularity = Math.abs(Math.cos(Math.atan2(p.getY() - r.getY(), p.getX() - r.getX()) - Math.atan2(p.getY() - this.getY(), p.getX() - this.getX())));
                    distanceSq = r.distanceSq(p);
                    thisrisk += energyratio * (1 + perpendicularity) / distanceSq;

                }
                if (thisrisk < minrisk) {
                    minrisk = thisrisk;
                    bestPoint = p;
                }
            }
        }
        return bestPoint;
    }

    /**
     * Genertiert mögliche Punkte im Bereich von 200 bis 300 Einheiten um den Bot die um eine safeDistance von der Wand entfernt sind.
     * @return die genertierten Punkte
     */
    public Point2D[] generate() {
        double height = this.getBattleFieldHeight();
        double width = this.getBattleFieldWidth();
        Point2D pointArray[];
        ArrayList points = new ArrayList(1);
        double theta;
        double dist;
        double safeDistance = 40;
        for (int i = 0; i < 200; i++) {
            theta = Math.random() * Math.PI * 2.0;
            dist = Math.random() * 100.0 + 200.0;
            Point2D p = projectPoint(new Point2D.Double(this.getX(), this.getY()), theta, dist);
            if (p.getX() > safeDistance && p.getX() < width - 2 * safeDistance && p.getY() > safeDistance && p.getY() < height - 2 * safeDistance) {
                points.add(p);
            }
        }
        pointArray = new Point2D[points.size()];
        for (int i = 0; i < points.size(); i++) {
            pointArray[i] = (Point2D) points.get(i);
        }
//        for (int i = 0; i< points.size(); i++){
//            System.out.println(a[i]);
//        }
        return pointArray;
    }
    
    /**
     * Trägt die Daten des gescannten Roboters in die Liste ein.
     *
     * @param e ScannedRobotEvent
     */
    @Override
    public void onScannedRobot(ScannedRobotEvent e) { //updated den Eintrag des gescanten Robots
        //http://robowiki.net/wiki/One_on_One_Radar
        if (this.singleEnemy){
            double radarTurn = getHeadingRadians() + e.getBearingRadians() - getRadarHeadingRadians();
            setTurnRadarRightRadians(1.9 * Utils.normalRelativeAngle(radarTurn));
        } else { //http://robowiki.net/wiki/Melee_Radar
            String name = e.getName();
            LinkedHashMap<String, Double> ehm = enemyHashMap;

            ehm.put(name, getHeadingRadians() + e.getBearingRadians());

            if ((name == sought || sought == null) && ehm.size() == getOthers()) {
                scanDir = Utils.normalRelativeAngle(ehm.values().iterator().next() - getRadarHeadingRadians());
                sought = ehm.keySet().iterator().next();
            }
        }

        //angelehnt an Shiz
        String scanName = e.getName();
        if (!robots.containsKey(scanName)) {
            RobotInfo robot = new RobotInfo();
            robot.NAME = scanName;
            robots.put(scanName, robot);
        }
        RobotInfo robot = (RobotInfo) robots.get(scanName);
        robot.energy = e.getEnergy();
        robot.velocity = e.getVelocity();
        robot.headingRad = e.getHeadingRadians();
        robot.bearingRad = e.getBearingRadians();
        robot.setLocation(projectPoint(new Point2D.Double(this.getX(), this.getY()), getHeadingRadians() + e.getBearingRadians(), e.getDistance()));
    }
    
    /**
     * Wählt ein neues Ziel aus.
     */
    private void chooseTarget() { //wählt nähesten Gegner wenn es momentan kein Ziel gibt; 
        //wechselt Ziel wenn ein Gegner dem Leader um 100 Einheiten näher ist als das momentan Ziel entfern ist
        //TO DO: zu schnelles Wechseln verhindern mit this.getTime()
        String closest = null;
        Point2D myPosition = new Point2D.Double(this.getY(), this.getX());
        double minDistance = Double.POSITIVE_INFINITY;
        double botDistance;
        double targetDistance;
        RobotInfo bot;
        if (target != null) {
            bot = (RobotInfo) robots.get(target);
            targetDistance = bot.distance(myPosition);
        } else {
            targetDistance = Double.POSITIVE_INFINITY;
        }
        for (String key : robots.keySet()){
            bot = (RobotInfo) robots.get(key);
            if (!this.isTeammate(bot.getName())) {
                botDistance = bot.distance(myPosition);
                if (botDistance < minDistance) {
                    minDistance = botDistance;
                    closest = bot.getName();
                }
            }
        }
        if (minDistance < targetDistance - 100.) {
            this.target = closest;
        }
        //System.out.println(this.target);
    }

    /**
     * Überprüfen ob eine Runde wegen zu langen Berechnungen übersprungen wurde.
     *
     * @param e SkippedTurnEvent
     */
    @Override
    public void onSkippedTurn(SkippedTurnEvent e) {
        System.out.println("Round " + e.getSkippedTurn() + " was skipped!");
    }

    /**
     * Löscht den Eintrag des getöteten Roboters aus der Liste und ruft
     * chooseTarget() auf wenn dieser das momentane Ziel war.
     *
     * @param e RobotDeathEvent
     */
    @Override
    public void onRobotDeath(RobotDeathEvent e) {
        String deadBot = e.getName();
        robots.remove(deadBot);
        if (deadBot.equals(target)) {
            target = null;
            chooseTarget();
        }
        sought = null;
    }

    /**
     * Sendet dem Droid seinen Tod und eine letzte neue Version der Liste.
     */
    public void onDeath() { //sendet dem Droid seinen Tod und eine letzte Info über alle Roboter
        robots.remove(this.getName());
        try {
            broadcastMessage("I'm dead!");
            broadcastMessage(robots);
        } catch (IOException ex) {
            Logger.getLogger(SteinbeißerLeader.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    /**
     * Berechnet einen Punkt aus einem Startpunkt, dem Winkel und dem Abstand.
     *
     * @param startPoint Startpunkt von dem gemessen wurde
     * @param theta Winkel zum Messpunkt
     * @param dist Entfernung zum Messpunkt
     * @return Berechneter Punkt
     */
    private static Point2D projectPoint(Point2D startPoint, double theta, double dist) { //von Shiz
        return new Point2D.Double(startPoint.getX() + dist * Math.sin(theta), startPoint.getY() + dist * Math.cos(theta));
    }

}
