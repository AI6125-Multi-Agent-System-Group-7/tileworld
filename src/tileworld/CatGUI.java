package tileworld;

import java.awt.AlphaComposite;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Composite;
import java.awt.Graphics2D;
import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.Map;

import javax.imageio.ImageIO;
import javax.swing.JFrame;

import sim.display.Console;
import sim.display.Controller;
import sim.display.Display2D;
import sim.display.GUIState;
import sim.engine.SimState;
import sim.field.grid.ObjectGrid2D;
import sim.portrayal.DrawInfo2D;
import sim.portrayal.SimplePortrayal2D;
import sim.portrayal.grid.ObjectGridPortrayal2D;
import tileworld.agent.TWAgent;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWObstacle;
import tileworld.environment.TWTile;

/**
 * CatGUI - Meow~~~
 *
 * A wrapper GUI that keeps the original model and logics..
 * But swaps in some cute-ness by over-nyanned original methods
 */
public class CatGUI extends GUIState {

    // ============= Kitty World Display Settings ============= //
    private static final int BASE_CELL_SIZE_IN_PIXELS = 44;
    private static final double DISPLAY_SCALE = 0.4;
    private static final int CELL_SIZE_IN_PIXELS =
            (int) Math.round(BASE_CELL_SIZE_IN_PIXELS * DISPLAY_SCALE);

    private static final Color BACKGROUND_COLOR = new Color(0xFA, 0xF5, 0xF0);
    private static final Color GRID_COLOR_A = new Color(0xFD, 0xF6, 0xF8);
    private static final Color GRID_COLOR_B = new Color(0xFA, 0xE9, 0xF2);
    private static final Color GRID_LINE_COLOR = new Color(0xF4, 0xD3, 0xE0, 110);

    public Display2D catsWorld;
    public JFrame catsBoundary;

    private final ObjectGridPortrayal2D backgroundGridPortrayal = new ObjectGridPortrayal2D();
    private final ObjectGridPortrayal2D objectGridPortrayal = new ObjectGridPortrayal2D();
    private final ObjectGridPortrayal2D agentGridPortrayal = new ObjectGridPortrayal2D();

    public CatGUI() {
        this(new TWEnvironment());
    }

    public CatGUI(SimState state) {
        super(state);
    }

    public static String getName() {
        return "Tileworld - Cat GUI";
    }

    public void setupPortrayals() {
        TWEnvironment tw = (TWEnvironment) state;

        ObjectGrid2D backgroundGrid = new ObjectGrid2D(tw.getxDimension(), tw.getyDimension());
        Object cell = new Object();
        for (int x = 0; x < tw.getxDimension(); x++) {
            for (int y = 0; y < tw.getyDimension(); y++) {
                backgroundGrid.set(x, y, cell);
            }
        }
        backgroundGridPortrayal.setField(backgroundGrid);
        backgroundGridPortrayal.setPortrayalForAll(
                new GridBackgroundPortrayal(GRID_COLOR_A, GRID_COLOR_B, GRID_LINE_COLOR));

        objectGridPortrayal.setField(tw.getObjectGrid());
        objectGridPortrayal.setPortrayalForClass(TWHole.class,
                new StaticImagePortrayal("assets/hole.png", new Color(0xE7, 0xC2, 0xC9)));
        objectGridPortrayal.setPortrayalForClass(TWTile.class,
                new StaticImagePortrayal("assets/yarn.png", new Color(0xD8, 0xB2, 0xC8)));
        objectGridPortrayal.setPortrayalForClass(TWFuelStation.class,
                new StaticImagePortrayal("assets/food.png", new Color(0xE9, 0xC8, 0xA6)));
        objectGridPortrayal.setPortrayalForClass(TWObstacle.class,
                new StaticImagePortrayal("assets/block.png", new Color(0xC6, 0xB0, 0xB5)));

        agentGridPortrayal.setField(tw.getAgentGrid());
        CatAgentPortrayal agentPortrayal = new CatAgentPortrayal();
        agentGridPortrayal.setPortrayalForClass(TWAgent.class, agentPortrayal);
        agentGridPortrayal.setPortrayalForRemainder(agentPortrayal);

        catsWorld.reset();
        catsWorld.repaint();
    }

    @Override
    public void start() {
        super.start();
        setupPortrayals();
    }

    @Override
    public void init(Controller c) {
        super.init(c);
        TWEnvironment tw = (TWEnvironment) state;
        catsWorld = new Display2D(
                tw.getxDimension() * CELL_SIZE_IN_PIXELS,
                tw.getyDimension() * CELL_SIZE_IN_PIXELS,
                this,
                1);

        catsBoundary = catsWorld.createFrame();
        c.registerFrame(catsBoundary);
        catsBoundary.setVisible(true);

        catsWorld.attach(backgroundGridPortrayal, "Background Grid");
        catsWorld.attach(objectGridPortrayal, "Tileworld Objects");
        catsWorld.attach(agentGridPortrayal, "Tileworld Agents");

        catsWorld.setBackdrop(BACKGROUND_COLOR);
    }

    // Meow Entry Point that instatiates the GUI
    public static void main(String[] args) {
        CatGUI gui = new CatGUI();
        Console c = new Console(gui);
        c.setVisible(true);
    }

    // Load cat assets
    private static final class CatAssets {
        private static final Map<String, Image> CACHE = new HashMap<String, Image>();

        private CatAssets() {
        }

        static Image getImage(String path) {
            Image cached = CACHE.get(path);
            if (cached != null) {
                return cached;
            }
            try {
                File file = new File(path);
                if (!file.exists()) {
                    return null;
                }
                Image img = ImageIO.read(file);
                CACHE.put(path, img);
                return img;
            } catch (IOException e) {
                return null;
            }
        }

        static Image getCatImage(int state, String dir) {
            String path = "assets/cat" + state + "_" + dir + ".png";
            return getImage(path);
        }
    }

    // Instead of Simple color blocks, display my pixart kitty
    private static final class StaticImagePortrayal extends SimplePortrayal2D {
        private final String path;
        private final Color fallbackColor;

        StaticImagePortrayal(String path, Color fallbackColor) {
            this.path = path;
            this.fallbackColor = fallbackColor;
        }

        @Override
        public void draw(Object object, Graphics2D graphics, DrawInfo2D info) {
            Image img = CatAssets.getImage(path);
            int x = (int) info.draw.x;
            int y = (int) info.draw.y;
            int w = (int) info.draw.width;
            int h = (int) info.draw.height;
            if (img != null) {
                graphics.drawImage(img, x, y, w, h, null);
            } else {
                graphics.setColor(fallbackColor);
                graphics.fillRect(x, y, w, h);
            }
        }
    }

    private static final class GridBackgroundPortrayal extends SimplePortrayal2D {
        private static final BasicStroke GRID_STROKE = new BasicStroke(1.0f);
        private final Color base;
        private final Color alt;
        private final Color line;

        GridBackgroundPortrayal(Color base, Color alt, Color line) {
            this.base = base;
            this.alt = alt;
            this.line = line;
        }

        @Override
        public void draw(Object object, Graphics2D graphics, DrawInfo2D info) {
            double cellW = info.draw.width;
            double cellH = info.draw.height;
            int cellX = 0;
            int cellY = 0;
            if (cellW > 0) {
                cellX = (int) Math.floor(info.draw.x / cellW);
            }
            if (cellH > 0) {
                cellY = (int) Math.floor(info.draw.y / cellH);
            }
            boolean useAlt = ((cellX + cellY) & 1) == 0;
            int x = (int) info.draw.x;
            int y = (int) info.draw.y;
            int w = (int) info.draw.width;
            int h = (int) info.draw.height;

            graphics.setColor(useAlt ? alt : base);
            graphics.fillRect(x, y, w, h);

            graphics.setStroke(GRID_STROKE);
            graphics.setColor(line);
            graphics.drawRect(x, y, w, h);
        }
    }

    private static final class CatAgentPortrayal extends SimplePortrayal2D {
        private static final Color[] PALETTE = new Color[] {
                new Color(0xF5, 0xA7, 0xC2),
                new Color(0x9E, 0xD6, 0xC4),
                new Color(0xF8, 0xC4, 0x8E),
                new Color(0xB5, 0xC7, 0xF2),
                new Color(0xE2, 0xB6, 0xE8)
        };

        private static final float TINT_ALPHA = 0.18f;

        private final Map<TWAgent, AgentState> stateMap = new IdentityHashMap<TWAgent, AgentState>();

        @Override
        public void draw(Object object, Graphics2D graphics, DrawInfo2D info) {
            if (!(object instanceof TWAgent)) {
                return;
            }

            TWAgent agent = (TWAgent) object;
            AgentState s = stateMap.get(agent);
            if (s == null) {
                s = new AgentState(agent.getX(), agent.getY(), "f");
                stateMap.put(agent, s);
            }

            int dx = agent.getX() - s.lastX;
            int dy = agent.getY() - s.lastY;
            if (dx != 0 || dy != 0) {
                if (dx > 0) {
                    s.dir = "r";
                } else if (dx < 0) {
                    s.dir = "l";
                } else if (dy > 0) {
                    s.dir = "f";
                } else if (dy < 0) {
                    s.dir = "b";
                }
                s.lastX = agent.getX();
                s.lastY = agent.getY();
            }

            int state = fuelState(agent);
            Image img = CatAssets.getCatImage(state, s.dir);

            int x = (int) info.draw.x;
            int y = (int) info.draw.y;
            int w = (int) info.draw.width;
            int h = (int) info.draw.height;

            if (img != null) {
                graphics.drawImage(img, x, y, w, h, null);
            } else {
                graphics.setColor(new Color(0x88, 0x88, 0x88));
                graphics.fillOval(x, y, w, h);
            }

            Color tint = agentColor(agent);
            Composite old = graphics.getComposite();
            graphics.setComposite(AlphaComposite.SrcOver.derive(TINT_ALPHA));
            graphics.setColor(tint);
            graphics.fillRect(x, y, w, h);
            graphics.setComposite(old);

            int badge = Math.max(4, Math.min(w, h) / 5);
            graphics.setColor(tint);
            graphics.fillOval(x + 2, y + 2, badge, badge);
        }

        private static int fuelState(TWAgent agent) {
            double maxFuel = Parameters.defaultFuelLevel;
            if (agent.getFuelLevel() <= 0) {
                return 4;
            }
            if (maxFuel <= 0) {
                return 1;
            }
            double ratio = agent.getFuelLevel() / maxFuel;
            if (ratio >= 0.5) {
                return 1;
            }
            if (ratio >= 0.25) {
                return 2;
            }
            return 3;
        }

        private static Color agentColor(TWAgent agent) {
            String name = agent.getName();
            int hash = name != null ? name.hashCode() : System.identityHashCode(agent);
            int idx = Math.abs(hash) % PALETTE.length;
            return PALETTE[idx];
        }

        private static final class AgentState {
            int lastX;
            int lastY;
            String dir;

            AgentState(int lastX, int lastY, String dir) {
                this.lastX = lastX;
                this.lastY = lastY;
                this.dir = dir;
            }
        }
    }
}

// By Hanny (Group 7) with only cats in mind \(^>w<^)/ \(^-w-^)/ \(^=w=^)/
// Last Edit: 11 Feb 2026
