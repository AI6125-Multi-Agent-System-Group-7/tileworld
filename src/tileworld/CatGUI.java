package tileworld;

import java.awt.BorderLayout;
import java.awt.AlphaComposite;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Component;
import java.awt.Composite;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Stroke;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.lang.reflect.Field;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

import javax.imageio.ImageIO;
import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.swing.ScrollPaneConstants;
import javax.swing.ToolTipManager;

import sim.display.Console;
import sim.display.Controller;
import sim.display.Display2D;
import sim.display.GUIState;
import sim.engine.SimState;
import sim.field.grid.ObjectGrid2D;
import sim.portrayal.DrawInfo2D;
import sim.portrayal.Inspector;
import sim.portrayal.LocationWrapper;
import sim.portrayal.SimplePortrayal2D;
import sim.portrayal.grid.ObjectGridPortrayal2D;
import sim.util.Int2D;
import tileworld.agent.TWAgent;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWObstacle;
import tileworld.environment.TWTile;

/**
 * CatGUI - Meow~~~World
 *
 * A wrapper GUI that keeps the original model and logics..
 * But swaps in some cute-ness by over-nyanned original methods
 */
public class CatGUI extends GUIState {

    // ============= Meow World Display Settings ============= //
    private static final int BASE_CELL_SIZE_IN_PIXELS = 44;
    private static final double DISPLAY_SCALE = 0.39;  // adjust to ur screen
    private static final int CELL_SIZE_IN_PIXELS =
            (int) Math.round(BASE_CELL_SIZE_IN_PIXELS * DISPLAY_SCALE);

    private static final Color BACKGROUND_COLOR = new Color(0xFA, 0xF5, 0xF0);
    private static final Color GRID_COLOR_A = new Color(0xFD, 0xF6, 0xF8);
    private static final Color GRID_COLOR_B = new Color(0xFA, 0xE9, 0xF2);
    private static final Color GRID_LINE_COLOR = new Color(0xF4, 0xD3, 0xE0, 110);
    private static final boolean HIDE_SCROLLBARS = true;
    private static int PAW_EDGE_PADDING = 0;
    private static final int PAW_RENDER_PAD = 0;
    private static final int TOOLTIP_INITIAL_DELAY_MS = 0;
    private static final float MEMORY_ALPHA_NORMAL = 0.7f;
    private static final float MEMORY_ALPHA_HOVERED = 0.95f;
    private static final float MEMORY_ALPHA_DIMMED = 0.30f;

    public Display2D catsWorld;
    public JFrame catsBoundary;

    private final ObjectGridPortrayal2D backgroundGridPortrayal = new ObjectGridPortrayal2D();
    private final ObjectGridPortrayal2D objectGridPortrayal = new ObjectGridPortrayal2D();
    private final ObjectGridPortrayal2D agentGridPortrayal = new ObjectGridPortrayal2D();
    private final List<MeowMoryLayer> memoryGridPortrayalList = new ArrayList<MeowMoryLayer>();
    private final Map<TWAgent, Integer> assetSetMap = new IdentityHashMap<TWAgent, Integer>();
    private int nextAssetSet = 1;
    private TWAgent hoveredAgent;
    private boolean hoverTrackingReady = false;

    public CatGUI() {
        this(new TWEnvironment());
    }

    public CatGUI(SimState state) {
        super(state);
    }

    public static String getName() {
        return "Tileworld - Cat GUI";
    }

    public static Object getInfo() {
        return "<H2>Moeworld - GUI</H2><p>Meow~ I turned Tileworld into a cozy kitty playroom. Follow the cats, watch the yarn, and purr at the catfoods. Have fun with this multi-cat system project!</p>";
    }

    public void setupDrawings() {
        TWEnvironment tw = (TWEnvironment) state;
        hoveredAgent = null;
        hoverTrackingReady = false;
        assetSetMap.clear();
        nextAssetSet = 1;

        ObjectGrid2D backgroundGrid = new ObjectGrid2D(tw.getxDimension(), tw.getyDimension());
        Object cell = new Object();
        for (int x = 0; x < tw.getxDimension(); x++) {
            for (int y = 0; y < tw.getyDimension(); y++) {
                backgroundGrid.set(x, y, cell);
            }
        }
        backgroundGridPortrayal.setField(backgroundGrid);
        backgroundGridPortrayal.setPortrayalForAll(
                new DrawMeowWorld(GRID_COLOR_A, GRID_COLOR_B, GRID_LINE_COLOR));

        objectGridPortrayal.setField(tw.getObjectGrid());
        objectGridPortrayal.setPortrayalForClass(TWHole.class,
                new DrawGameAsset("assets/hole.png", new Color(0xE7, 0xC2, 0xC9)));
        objectGridPortrayal.setPortrayalForClass(TWTile.class,
                new DrawGameAsset("assets/yarn.png", new Color(0xD8, 0xB2, 0xC8)));
        objectGridPortrayal.setPortrayalForClass(TWFuelStation.class,
                new DrawGameAsset("assets/food.png", new Color(0xE9, 0xC8, 0xA6)));
        objectGridPortrayal.setPortrayalForClass(TWObstacle.class,
                new DrawGameAsset("assets/block.png", new Color(0xC6, 0xB0, 0xB5)));

        agentGridPortrayal.setField(tw.getAgentGrid());
        DrawMeowMeow agentPortrayal = new DrawMeowMeow(this);
        agentGridPortrayal.setPortrayalForClass(TWAgent.class, agentPortrayal);
        agentGridPortrayal.setPortrayalForRemainder(agentPortrayal);

        catsWorld.reset();
        catsWorld.repaint();
    }

    private void setupMeowMoryDrawings() {
        memoryGridPortrayalList.clear();
        TWEnvironment tw = (TWEnvironment) state;
        ObjectGrid2D agentGrid = tw.getAgentGrid();
        for (int x = 0; x < tw.getxDimension(); x++) {
            for (int y = 0; y < tw.getyDimension(); y++) {
                Object obj = agentGrid.get(x, y);
                if (obj instanceof TWAgent) {
                    addMeowMoryPaws((TWAgent) obj);
                }
            }
        }
        resetDisplay();
    }

    // Kitty will left a paw print on remembered cells
    private void addMeowMoryPaws(TWAgent agent) {
        ObjectGridPortrayal2D memoryPortrayal = new ObjectGridPortrayal2D();
        memoryPortrayal.setField(agent.getMemory().getMemoryGrid());
        int spriteSet = assetSetFor(agent);
        String pawPath = CatAssets.getPawPath(agent.getName(), spriteSet);
        memoryPortrayal.setPortrayalForNonNull(
                new DrawMemoryPaw(this, agent, pawPath, new Color(0xE9, 0xC9, 0xD7)));
        memoryGridPortrayalList.add(new MeowMoryLayer(memoryPortrayal, agent.getName() + "'s Memory"));
    }

    public void resetDisplay() {
        catsWorld.detatchAll();
        catsWorld.attach(backgroundGridPortrayal, "Background Grid");
        catsWorld.attach(objectGridPortrayal, "Tileworld Objects");
        catsWorld.attach(agentGridPortrayal, "Tileworld Agents");
        for (MeowMoryLayer layer : memoryGridPortrayalList) {
            catsWorld.attach(layer.portrayal, layer.label);
        }
        catsWorld.setBackdrop(BACKGROUND_COLOR);
    }

    @Override
    public void start() {
        super.start();
        setupDrawings();
        setupMeowMoryDrawings();
    }

    @Override
    public void init(Controller c) {
        super.init(c);
        TWEnvironment tw = (TWEnvironment) state;
        catsWorld = new Display2D(
                tw.getxDimension() * CELL_SIZE_IN_PIXELS + PAW_RENDER_PAD,
                tw.getyDimension() * CELL_SIZE_IN_PIXELS + PAW_RENDER_PAD,
                this,
                1);

        catsBoundary = catsWorld.createFrame();
        c.registerFrame(catsBoundary);
        catsBoundary.setVisible(true);

        catsWorld.attach(backgroundGridPortrayal, "Background Grid");
        catsWorld.attach(objectGridPortrayal, "Tileworld Objects");
        catsWorld.attach(agentGridPortrayal, "Tileworld Agents");

        catsWorld.setBackdrop(BACKGROUND_COLOR);
        catsWorld.useTooltips = true;
        ToolTipManager.sharedInstance().setInitialDelay(TOOLTIP_INITIAL_DELAY_MS);
        tuneScrollPaws();
        installHoverTracking();
    }

    // Meow Entry Point that instatiates the GUI
    public static void main(String[] args) {
        CatGUI gui = new CatGUI();
        Console c = new Console(gui);
        c.setVisible(true);
    }

    @Override
    public void quit() {
        super.quit();
        if (catsBoundary != null) {
            catsBoundary.dispose();
        }
        catsBoundary = null;
        catsWorld = null;
        System.out.println("Final reward: " + ((TWEnvironment) state).getReward());
    }

    private void tuneScrollPaws() {
        if (!HIDE_SCROLLBARS) {
            return;
        }
        try {
            JScrollPane pane = getDisplayPane();
            if (pane == null) {
                return;
            }
            pane.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
            pane.setVerticalScrollBarPolicy(ScrollPaneConstants.VERTICAL_SCROLLBAR_NEVER);
            pane.setBorder(BorderFactory.createEmptyBorder());
            pane.setViewportBorder(null);
            if (PAW_EDGE_PADDING > 0) {
                pane.setViewportBorder(BorderFactory.createEmptyBorder(
                        0, 0, PAW_EDGE_PADDING, PAW_EDGE_PADDING));
            }
        } catch (Exception e) {
            // I'll handle this later, ignored for now
        }
    }

    private JScrollPane getDisplayPane() {
        try {
            Field field = Display2D.class.getDeclaredField("display");
            field.setAccessible(true);
            return (JScrollPane) field.get(catsWorld);
        } catch (Exception e) {
            return null;
        }
    }

    private void installHoverTracking() {
        if (hoverTrackingReady || catsWorld == null) {
            return;
        }
        JScrollPane pane = getDisplayPane();
        if (pane == null || pane.getViewport() == null) {
            return;
        }
        final Component view = pane.getViewport().getView();
        if (view == null) {
            return;
        }
        MouseAdapter hoverHandler = new MouseAdapter() {
            @Override
            public void mouseMoved(MouseEvent e) {
                updateHoveredAgent(e.getX(), e.getY(), view);
            }

            @Override
            public void mouseDragged(MouseEvent e) {
                updateHoveredAgent(e.getX(), e.getY(), view);
            }

            @Override
            public void mouseExited(MouseEvent e) {
                setHoveredAgent(null);
            }
        };
        view.addMouseMotionListener(hoverHandler);
        view.addMouseListener(hoverHandler);
        hoverTrackingReady = true;
    }

    private void updateHoveredAgent(int mouseX, int mouseY, Component view) {
        if (!(state instanceof TWEnvironment)) {
            setHoveredAgent(null);
            return;
        }
        TWEnvironment tw = (TWEnvironment) state;
        int xDim = tw.getxDimension();
        int yDim = tw.getyDimension();
        if (xDim <= 0 || yDim <= 0) {
            setHoveredAgent(null);
            return;
        }

        double cellW = Math.max(1.0, (double) view.getWidth() / xDim);
        double cellH = Math.max(1.0, (double) view.getHeight() / yDim);
        int gridX = (int) Math.floor(mouseX / cellW);
        int gridY = (int) Math.floor(mouseY / cellH);
        if (gridX < 0 || gridX >= xDim || gridY < 0 || gridY >= yDim) {
            setHoveredAgent(null);
            return;
        }
        ObjectGrid2D agentGrid = tw.getAgentGrid();
        if (agentGrid == null) {
            setHoveredAgent(null);
            return;
        }
        Object obj = agentGrid.get(gridX, gridY);
        if (obj instanceof TWAgent) {
            setHoveredAgent((TWAgent) obj);
            return;
        }
        setHoveredAgent(null);
    }

    private void setHoveredAgent(TWAgent agent) {
        if (hoveredAgent == agent) {
            return;
        }
        hoveredAgent = agent;
        if (catsWorld != null) {
            catsWorld.repaint();
        }
    }

    private float memoryAlphaFor(TWAgent owner) {
        if (hoveredAgent == null) {
            return MEMORY_ALPHA_NORMAL;
        }
        if (hoveredAgent == owner) {
            return MEMORY_ALPHA_HOVERED;
        }
        return MEMORY_ALPHA_DIMMED;
    }

    private int assetSetFor(TWAgent agent) {
        Integer idx = assetSetMap.get(agent);
        if (idx != null) {
            return idx.intValue();
        }
        int assigned = nextAssetSet;
        nextAssetSet = (nextAssetSet % 6) + 1;
        assetSetMap.put(agent, Integer.valueOf(assigned));
        return assigned;
    }

    // Load assets from the naming style I defined (agent#/cat#_#.png, agentNum, state, direction)
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

        static Image getCatIllust(String agentName, int agentIndex, int state, String dir) {
            if (agentName != null) {
                String normalized = agentName.trim();
                if ("github".equalsIgnoreCase(normalized) || "octocat".equalsIgnoreCase(normalized)) {
                    String octoPath = "assets/octocat/cat" + state + "_" + dir + ".png";
                    return getImage(octoPath);
                }
            }
            int idx = Math.max(1, Math.min(agentIndex, 6));
            String path = "assets/agent" + idx + "/cat" + state + "_" + dir + ".png";
            return getImage(path);
        }

        static String getPawPath(String agentName, int agentIndex) {
            if (agentName != null) {
                String normalized = agentName.trim();
                if ("github".equalsIgnoreCase(normalized) || "octocat".equalsIgnoreCase(normalized)) {
                    return "assets/octocat/paw.png";
                }
            }
            int idx = Math.max(1, Math.min(agentIndex, 6));
            return "assets/agent" + idx + "/paw.png";
        }
    }

    // Instead of Simple color blocks, display my pixart objects and kittens
    private static final class DrawGameAsset extends SimplePortrayal2D {
        private final String path;
        private final Color fallbackColor;

        DrawGameAsset(String path, Color fallbackColor) {
            this.path = path;
            this.fallbackColor = fallbackColor;
        }

        @Override
        public void draw(Object object, Graphics2D graphics, DrawInfo2D info) {
            Image img = CatAssets.getImage(path);
            int w = (int) Math.round(info.draw.width);
            int h = (int) Math.round(info.draw.height);
            int x = (int) Math.round(info.draw.x - w / 2.0);
            int y = (int) Math.round(info.draw.y - h / 2.0);
            if (img != null) {
                graphics.drawImage(img, x, y, w, h, null);
            } else {
                graphics.setColor(fallbackColor);
                graphics.fillRect(x, y, w, h);
            }
        }
    }

    private static final class DrawMeowWorld extends SimplePortrayal2D {
        private static final BasicStroke GRID_STROKE = new BasicStroke(1.0f);
        private final Color base;
        private final Color alt;
        private final Color line;

        DrawMeowWorld(Color base, Color alt, Color line) {
            this.base = base;
            this.alt = alt;
            this.line = line;
        }

        @Override
        public void draw(Object object, Graphics2D graphics, DrawInfo2D info) {
            double cellW = info.draw.width;
            double cellH = info.draw.height;
            double drawX = info.draw.x - cellW / 2.0;
            double drawY = info.draw.y - cellH / 2.0;
            int cellX = 0;
            int cellY = 0;
            if (cellW > 0) {
                cellX = (int) Math.floor(drawX / cellW);
            }
            if (cellH > 0) {
                cellY = (int) Math.floor(drawY / cellH);
            }
            boolean useAlt = ((cellX + cellY) & 1) == 0;
            int x = (int) Math.round(drawX);
            int y = (int) Math.round(drawY);
            int w = (int) Math.round(cellW);
            int h = (int) Math.round(cellH);

            graphics.setColor(useAlt ? alt : base);
            graphics.fillRect(x, y, w, h);

            graphics.setStroke(GRID_STROKE);
            graphics.setColor(line);
            graphics.drawRect(x, y, w, h);
        }
    }

    private final class DrawMemoryPaw extends SimplePortrayal2D {
        private final CatGUI gui;
        private final TWAgent owner;
        private final String path;
        private final Color fallbackColor;

        DrawMemoryPaw(CatGUI gui, TWAgent owner, String path, Color fallbackColor) {
            this.gui = gui;
            this.owner = owner;
            this.path = path;
            this.fallbackColor = fallbackColor;
        }

        @Override
        public void draw(Object object, Graphics2D graphics, DrawInfo2D info) {
            float alpha = gui.memoryAlphaFor(owner);
            alpha = Math.max(0.0f, Math.min(1.0f, alpha));
            Composite oldComposite = graphics.getComposite();
            graphics.setComposite(AlphaComposite.SrcOver.derive(alpha));

            Image img = CatAssets.getImage(path);
            int w = (int) Math.round(info.draw.width);
            int h = (int) Math.round(info.draw.height);
            int x = (int) Math.round(info.draw.x - w / 2.0);
            int y = (int) Math.round(info.draw.y - h / 2.0);
            if (img != null) {
                graphics.drawImage(img, x, y, w, h, null);
            } else {
                graphics.setColor(fallbackColor);
                graphics.fillRect(x, y, w, h);
            }

            graphics.setComposite(oldComposite);
        }
    }

    private static final class DrawMeowMeow extends SimplePortrayal2D {
        private static final Color[] PALETTE = new Color[] {
                new Color(0xF5, 0xA7, 0xC2),
                new Color(0x9E, 0xD6, 0xC4),
                new Color(0xF8, 0xC4, 0x8E),
                new Color(0xB5, 0xC7, 0xF2),
                new Color(0xE2, 0xB6, 0xE8)
        };

        private static final float TINT_ALPHA = 0.18f;
        private static final Color SENSOR_BORDER_COLOR = new Color(0xE5, 0x4B, 0x8F, 190);
        private static final BasicStroke SENSOR_BORDER_STROKE = new BasicStroke(
                1.4f,
                BasicStroke.CAP_ROUND,
                BasicStroke.JOIN_ROUND,
                1f,
                new float[] {4f, 3f},
                0f);

        private final CatGUI gui;
        private final Map<TWAgent, AgentState> stateMap = new IdentityHashMap<TWAgent, AgentState>();

        DrawMeowMeow(CatGUI gui) {
            this.gui = gui;
        }

        @Override
        public boolean hitObject(Object object, DrawInfo2D range) {
            final double width = range.draw.width;
            final double height = range.draw.height;
            return range.clip.intersects(
                    range.draw.x - width / 2.0,
                    range.draw.y - height / 2.0,
                    width,
                    height);
        }

        @Override
        public Inspector getInspector(LocationWrapper wrapper, GUIState state) {
            return new CatAgentInspector(super.getInspector(wrapper, state), wrapper, state);
        }

        @Override
        public String getStatus(LocationWrapper wrapper) {
            Object obj = wrapper != null ? wrapper.getObject() : null;
            if (!(obj instanceof TWAgent)) {
                return null;
            }
            TWAgent agent = (TWAgent) obj;
            String name = agent.getName();
            if (name == null || name.trim().isEmpty()) {
                name = "Agent";
            }
            return name + " | Score: " + agent.getScore() + " | Fuel: " + (int) agent.getFuelLevel() + " | Pos: (" + agent.getX() + "," + agent.getY() + ")";
        }

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
            int spriteSet = gui.assetSetFor(agent);
            Image img = CatAssets.getCatIllust(agent.getName(), spriteSet, state, s.dir);

            int w = (int) Math.round(info.draw.width);
            int h = (int) Math.round(info.draw.height);
            int x = (int) Math.round(info.draw.x - w / 2.0);
            int y = (int) Math.round(info.draw.y - h / 2.0);

            if (img != null) {
                graphics.drawImage(img, x, y, w, h, null);
            } else {
                graphics.setColor(new Color(0x88, 0x88, 0x88));
                graphics.fillOval(x, y, w, h);
            }

            Color tint = agentColor(agent);
            Composite oldComposite = graphics.getComposite();
            graphics.setComposite(AlphaComposite.SrcOver.derive(TINT_ALPHA));
            graphics.setColor(tint);
            graphics.fillRect(x, y, w, h);
            graphics.setComposite(oldComposite);

            int badge = Math.max(4, Math.min(w, h) / 5);
            graphics.setColor(tint);
            graphics.fillOval(x + 2, y + 2, badge, badge);

            int range = Parameters.defaultSensorRange;
            if (range > 0) {
                int rx = x - (int) (range * w);
                int ry = y - (int) (range * h);
                int rw = (int) (w * (2 * range + 1));
                int rh = (int) (h * (2 * range + 1));
                Stroke oldStroke = graphics.getStroke();
                graphics.setStroke(SENSOR_BORDER_STROKE);
                graphics.setColor(SENSOR_BORDER_COLOR);
                graphics.drawRect(rx, ry, rw, rh);
                graphics.setStroke(oldStroke);
            }
        }

        private static int fuelState(TWAgent agent) {
            double maxFuel = Parameters.defaultFuelLevel;
            if (agent.getFuelLevel() <= 0) {
                return 3;
            }
            if (maxFuel <= 0) {
                return 1;
            }
            double ratio = agent.getFuelLevel() / maxFuel;
            if (ratio >= 0.5) {
                return 1;
            }
            return 2;
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

    private static final class CatAgentInspector extends Inspector {
        private final Inspector originalInspector;

        CatAgentInspector(Inspector originalInspector, LocationWrapper wrapper, GUIState guiState) {
            this.originalInspector = originalInspector;

            final TWAgent agent = (TWAgent) wrapper.getObject();
            final SimState simState = guiState.state;
            final Controller console = guiState.controller;

            Box box = new Box(BoxLayout.X_AXIS);
            JButton button = new JButton("Teleport Agent");
            box.add(button);
            box.add(Box.createGlue());

            setLayout(new BorderLayout());
            add(originalInspector, BorderLayout.CENTER);
            add(box, BorderLayout.SOUTH);

            button.addActionListener(new ActionListener() {
                public void actionPerformed(ActionEvent e) {
                    synchronized (simState.schedule) {
                        Int2D loc = agent.getEnvironment().generateRandomLocation();
                        agent.getEnvironment().getAgentGrid().set(agent.getX(), agent.getY(), null);
                        agent.setLocation(loc);
                        if (console != null) {
                            console.refresh();
                        }
                    }
                }
            });
        }

        @Override
        public void updateInspector() {
            originalInspector.updateInspector();
        }
    }

    private static final class MeowMoryLayer {
        final ObjectGridPortrayal2D portrayal;
        final String label;

        MeowMoryLayer(ObjectGridPortrayal2D portrayal, String label) {
            this.portrayal = portrayal;
            this.label = label;
        }
    }
}

// By Hanny (Group 7) with only kitties in mind \(^>w<^)/ \(^-w-^)/ \(^=w=^)/
// Last Edit: 01 Mar 2026
