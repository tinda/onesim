/* 
 * Copyright 2010 Aalto University, ComNet
 * Released under GPLv3. See LICENSE.txt for details. 
 */
package movement;

import input.WKTMapReader;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.Vector;

import movement.MapBasedMovement.MyData;
import movement.map.MapNode;
import movement.map.SimMap;
import core.Coord;
import core.Settings;
import core.SettingsError;
import core.SimClock;
import core.SimError;

/**
 * Map based movement model which gives out Paths that use the roads of a
 * SimMap.
 */
public class MapBasedMovement extends MovementModel implements
		SwitchableMovement {
	/** sim map for the model */
	private SimMap map = null;
	/** node where the last path ended or node next to initial placement */
	protected MapNode lastMapNode;
	/** max nrof map nodes to travel/path */
	protected int maxPathLength = 100;
	/** min nrof map nodes to travel/path */
	protected int minPathLength = 100;
	/** May a node choose to move back the same way it came at a crossing */
	protected boolean backAllowed;
	/** map based movement model's settings namespace ({@value} ) */
	public static final String MAP_BASE_MOVEMENT_NS = "MapBasedMovement";
	/** number of map files -setting id ({@value} ) */
	public static final String NROF_FILES_S = "nrofMapFiles";
	/** map file -setting id ({@value} ) */
	public static final String FILE_S = "mapFile";

	/**
	 * Per node group setting for selecting map node types that are OK for this
	 * node group to traverse trough. Value must be a comma separated list of
	 * integers in range of [1,31]. Values reference to map file indexes (see
	 * {@link #FILE_S}). If setting is not defined, all map nodes are considered
	 * OK.
	 */
	public static final String MAP_SELECT_S = "okMaps";

	/** the indexes of the OK map files or null if all maps are OK */
	private int[] okMapNodeTypes;

	/** how many map files are read */
	private int nrofMapFilesRead = 0;
	/** map cache -- in case last mm read the same map, use it without loading */
	private static SimMap cachedMap = null;
	/** names of the previously cached map's files (for hit comparison) */
	private static List<String> cachedMapFiles = null;
	/** store all of road from & to */
	public static ArrayList<MyData> lst = new ArrayList<MyData>();

	/**
	 * Creates a new MapBasedMovement based on a Settings object's settings.
	 * 
	 * @param settings
	 *            The Settings object where the settings are read from
	 */
	public MapBasedMovement(Settings settings) {
		super(settings);
		map = readMap();
		readOkMapNodeTypes(settings);
		maxPathLength = 100;
		minPathLength = 10;
		backAllowed = false;
		ReadMap();
	}

	/**
	 * Creates a new MapBasedMovement based on a Settings object's settings but
	 * with different SimMap
	 * 
	 * @param settings
	 *            The Settings object where the settings are read from
	 * @param newMap
	 *            The SimMap to use
	 * @param nrofMaps
	 *            How many map "files" are in the map
	 */
	public MapBasedMovement(Settings settings, SimMap newMap, int nrofMaps) {
		super(settings);
		map = newMap;
		this.nrofMapFilesRead = nrofMaps;
		readOkMapNodeTypes(settings);
		maxPathLength = 100;
		minPathLength = 10;
		backAllowed = false;
		ReadMap();
	}

	class MyData {
		String id = null;
		String level = null;
		double fx = 0.0;
		double fy = 0.0;
		double tx = 0.0;
		double ty = 0.0;

		MyData(String line) {
			String[] token = line.split(" ");
			id = token[0];
			level = token[5];
			fx = Double.parseDouble(token[1]);
			fy = Double.parseDouble(token[2]);
			tx = Double.parseDouble(token[3]);
			ty = Double.parseDouble(token[4]);

		}
	}

	public void ReadMap() {

		BufferedReader br = null;
		try {
			br = new BufferedReader(new InputStreamReader(new FileInputStream(
					"C:\\Users\\ncue\\Documents\\sim\\road\\road level.txt")));
			String line = null;
			while ((line = br.readLine()) != null) {
				lst.add(new MyData(line));
			}
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	/**
	 * Reads the OK map node types from settings
	 * 
	 * @param settings
	 *            The settings where the types are read
	 */
	private void readOkMapNodeTypes(Settings settings) {
		if (settings.contains(MAP_SELECT_S)) {
			this.okMapNodeTypes = settings.getCsvInts(MAP_SELECT_S);
			for (int i : okMapNodeTypes) {
				if (i < MapNode.MIN_TYPE || i > MapNode.MAX_TYPE) {
					throw new SettingsError("Map type selection '" + i
							+ "' is out of range for setting "
							+ settings.getFullPropertyName(MAP_SELECT_S));
				}
				if (i > nrofMapFilesRead) {
					throw new SettingsError("Can't use map type selection '"
							+ i + "' for setting "
							+ settings.getFullPropertyName(MAP_SELECT_S)
							+ " because only " + nrofMapFilesRead
							+ " map files are read");
				}
			}
		} else {
			this.okMapNodeTypes = null;
		}
	}

	/**
	 * Copyconstructor.
	 * 
	 * @param mbm
	 *            The MapBasedMovement object to base the new object to
	 */
	protected MapBasedMovement(MapBasedMovement mbm) {
		super(mbm);
		this.okMapNodeTypes = mbm.okMapNodeTypes;
		this.map = mbm.map;
		this.minPathLength = mbm.minPathLength;
		this.maxPathLength = mbm.maxPathLength;
		this.backAllowed = mbm.backAllowed;
	}

	/**
	 * Returns a (random) coordinate that is between two adjacent MapNodes
	 */
	@Override
	public Coord getInitialLocation() {
		List<MapNode> nodes = map.getNodes();
		MapNode n, n2;
		Coord n2Location, nLocation, placement;
		double dx, dy;
		double rnd = rng.nextDouble();

		// choose a random node (from OK types if such are defined)
		do {
			n = nodes.get(rng.nextInt(nodes.size()));
		} while (okMapNodeTypes != null && !n.isType(okMapNodeTypes));

		// choose a random neighbor of the selected node
		n2 = n.getNeighbors().get(rng.nextInt(n.getNeighbors().size()));

		nLocation = n.getLocation();
		n2Location = n2.getLocation();

		placement = n.getLocation().clone();

		dx = rnd * (n2Location.getX() - nLocation.getX());
		dy = rnd * (n2Location.getY() - nLocation.getY());

		placement.translate(dx, dy); // move coord from n towards n2

		this.lastMapNode = n;
		return placement;
	}

	/**
	 * Returns map node types that are OK for this movement model in an array or
	 * null if all values are considered ok
	 * 
	 * @return map node types that are OK for this movement model in an array
	 */
	protected int[] getOkMapNodeTypes() {
		return okMapNodeTypes;
	}
	
	private double getSimTime() {
		// TODO Auto-generated method stub
		return SimClock.getTime();
	}

	@Override
	public Path getPath() {
		NumberFormat format = new DecimalFormat("#0.000000");
		double time = getSimTime();
		double speed = generateSpeed();
		Path p = new Path(speed);
		MapNode curNode = lastMapNode;
		MapNode prevNode = lastMapNode;
		MapNode nextNode = null;
		List<MapNode> neighbors;
		Coord nextCoord;
		Double limit = speed;

		for (int i = 0; i < lst.size(); i++) {
			if ((lst.get(i).fx < (curNode.getLocation().getX() + 300995.01117))
					&& ((curNode.getLocation().getX() + 300995.01117) < lst.get(i).tx)
					&& (lst.get(i).fy < (2773434.9968 - curNode.getLocation().getY()))
					&& ((2773434.9968 - curNode.getLocation().getY()) < lst.get(i).ty)) {
//				System.out.println("W"+curNode.getLocation().getX()+"\t"+curNode.getLocation().getY());
				if(lst.get(i).level.equals("\tHW")||lst.get(i).level.equals("\tHU")){
					limit = 90*0.277777778;
				}else if(lst.get(i).level.equals("\t1E")||lst.get(i).level.equals("\t1W")||lst.get(i).level.equals("\t1U")||lst.get(i).level.equals("\tRE")){
					limit = 70*0.277777778;
				}else if(lst.get(i).level.equals("\t2W")||lst.get(i).level.equals("\t2U")||lst.get(i).level.equals("\tRD")||lst.get(i).level.equals("\t3W")||lst.get(i).level.equals("\t3U")){
					limit = 50*0.277777778;
				}else if(lst.get(i).level.equals("\tOR")||lst.get(i).level.equals("\tOT")||lst.get(i).level.equals("\t4W")||lst.get(i).level.equals("\tAL")){
					limit = 40*0.277777778;
				}else{
					limit = 40*0.277777778;
				}

			}
		}
		if(((time>7*3600)&&(time<=9*3600))||((time>12*3600)&&(time<=13*3600))||((time>17*3600)&&(time<=19*3600))||((time>22*3600)&&(time<=23*3600))){
			limit=limit*0.6;
		}
		assert lastMapNode != null : "Tried to get a path before placement";
		double x = (Math.random()*5);
		int y = (int) (Math.random() * 10);
		
		// start paths from current node
		if (y % 2 == 0) {
			if((speed + x)>limit){
				p.addWaypoint(curNode.getLocation(), limit);
			}else if((speed + x)<limit){
				p.addWaypoint(curNode.getLocation(), speed + x);
			}else{
				p.addWaypoint(curNode.getLocation(), speed);
			}
		} else {
			if((speed - x)>limit){
				p.addWaypoint(curNode.getLocation(), limit);
			}else if((speed - x)<limit){
				if(speed - x <0){
					p.addWaypoint(curNode.getLocation(), speed );
				}else{
					p.addWaypoint(curNode.getLocation(), speed - x);
				}
			}else{
				p.addWaypoint(curNode.getLocation(), speed);
			}
		}

		int pathLength = rng.nextInt(maxPathLength - minPathLength)
				+ minPathLength;

		for (int i = 0; i < pathLength; i++) {
			neighbors = curNode.getNeighbors();
			Vector<MapNode> n2 = new Vector<MapNode>(neighbors);
			if (!this.backAllowed) {
				n2.remove(prevNode); // to prevent going back
			}

			if (okMapNodeTypes != null) { // remove neighbor nodes that aren't
											// ok
				for (int j = 0; j < n2.size();) {
					if (!n2.get(j).isType(okMapNodeTypes)) {
						n2.remove(j);
					} else {
						j++;
					}
				}
			}

			if (n2.size() == 0) { // only option is to go back
				nextNode = prevNode;
			} else { // choose a random node from remaining neighbors
				nextNode = n2.get(rng.nextInt(n2.size()));
			}

			prevNode = curNode;

			nextCoord = nextNode.getLocation();
			curNode = nextNode;
			
			if((speed + x)>limit){
				p.addWaypoint(curNode.getLocation(), limit);
			}else if((speed + x)<limit){
				p.addWaypoint(curNode.getLocation(), speed + x);
			}else{
				p.addWaypoint(curNode.getLocation(), speed);
			}
		}

		lastMapNode = curNode;
		limit = speed;
		return p;
	}

	/**
	 * Selects and returns a random node that is OK from a list of nodes.
	 * Whether node is OK, is determined by the okMapNodeTypes list. If
	 * okMapNodeTypes are defined, the given list <strong>must</strong> contain
	 * at least one OK node to prevent infinite looping.
	 * 
	 * @param nodes
	 *            The list of nodes to choose from.
	 * @return A random node from the list (that is OK if ok list is defined)
	 */
	protected MapNode selectRandomOkNode(List<MapNode> nodes) {
		MapNode n;
		do {
			n = nodes.get(rng.nextInt(nodes.size()));
		} while (okMapNodeTypes != null && !n.isType(okMapNodeTypes));

		return n;
	}

	/**
	 * Returns the SimMap this movement model uses
	 * 
	 * @return The SimMap this movement model uses
	 */
	public SimMap getMap() {
		return map;
	}

	/**
	 * Reads a sim map from location set to the settings, mirrors the map and
	 * moves its upper left corner to origo.
	 * 
	 * @return A new SimMap based on the settings
	 */
	private SimMap readMap() {
		SimMap simMap;
		Settings settings = new Settings(MAP_BASE_MOVEMENT_NS);
		WKTMapReader r = new WKTMapReader(true);

		if (cachedMap == null) {
			cachedMapFiles = new ArrayList<String>(); // no cache present
		} else { // something in cache
					// check out if previously asked map was asked again
			SimMap cached = checkCache(settings);
			if (cached != null) {
				nrofMapFilesRead = cachedMapFiles.size();
				return cached; // we had right map cached -> return it
			} else { // no hit -> reset cache
				cachedMapFiles = new ArrayList<String>();
				cachedMap = null;
			}
		}

		try {
			int nrofMapFiles = settings.getInt(NROF_FILES_S);

			for (int i = 1; i <= nrofMapFiles; i++) {
				String pathFile = settings.getSetting(FILE_S + i);
				cachedMapFiles.add(pathFile);
				r.addPaths(new File(pathFile), i);
			}

			nrofMapFilesRead = nrofMapFiles;
		} catch (IOException e) {
			throw new SimError(e.toString(), e);
		}

		simMap = r.getMap();
		checkMapConnectedness(simMap.getNodes());
		// mirrors the map (y' = -y) and moves its upper left corner to origo
		simMap.mirror();
		Coord offset = simMap.getMinBound().clone();
		simMap.translate(-offset.getX(), -offset.getY());
		checkCoordValidity(simMap.getNodes());

		cachedMap = simMap;
		return simMap;
	}

	/**
	 * Checks that all map nodes can be reached from all other map nodes
	 * 
	 * @param nodes
	 *            The list of nodes to check
	 * @throws SettingsError
	 *             if all map nodes are not connected
	 */
	private void checkMapConnectedness(List<MapNode> nodes) {
		Set<MapNode> visited = new HashSet<MapNode>();
		Queue<MapNode> unvisited = new LinkedList<MapNode>();
		MapNode firstNode;
		MapNode next = null;

		if (nodes.size() == 0) {
			throw new SimError("No map nodes in the given map");
		}

		firstNode = nodes.get(0);

		visited.add(firstNode);
		unvisited.addAll(firstNode.getNeighbors());

		while ((next = unvisited.poll()) != null) {
			visited.add(next);
			for (MapNode n : next.getNeighbors()) {
				if (!visited.contains(n) && !unvisited.contains(n)) {
					unvisited.add(n);
				}
			}
		}

		if (visited.size() != nodes.size()) { // some node couldn't be reached
			MapNode disconnected = null;
			for (MapNode n : nodes) { // find an example node
				if (!visited.contains(n)) {
					disconnected = n;
					break;
				}
			}
			throw new SettingsError("SimMap is not fully connected. Only "
					+ visited.size() + " out of " + nodes.size()
					+ " map nodes " + "can be reached from " + firstNode
					+ ". E.g. " + disconnected + " can't be reached");
		}
	}

	/**
	 * Checks that all coordinates of map nodes are within the min&max limits of
	 * the movement model
	 * 
	 * @param nodes
	 *            The list of nodes to check
	 * @throws SettingsError
	 *             if some map node is out of bounds
	 */
	private void checkCoordValidity(List<MapNode> nodes) {
		// Check that all map nodes are within world limits
		for (MapNode n : nodes) {
			double x = n.getLocation().getX();
			double y = n.getLocation().getY();
			if (x < 0 || x > getMaxX() || y < 0 || y > getMaxY()) {
				throw new SettingsError("Map node " + n.getLocation()
						+ " is out of world  bounds " + "(x: 0..." + getMaxX()
						+ " y: 0..." + getMaxY() + ")");
			}
		}
	}

	/**
	 * Checks map cache if the requested map file(s) match to the cached sim map
	 * 
	 * @param settings
	 *            The Settings where map file names are found
	 * @return A cached map or null if the cached map didn't match
	 */
	private SimMap checkCache(Settings settings) {
		int nrofMapFiles = settings.getInt(NROF_FILES_S);

		if (nrofMapFiles != cachedMapFiles.size() || cachedMap == null) {
			return null; // wrong number of files
		}

		for (int i = 1; i <= nrofMapFiles; i++) {
			String pathFile = settings.getSetting(FILE_S + i);
			if (!pathFile.equals(cachedMapFiles.get(i - 1))) {
				return null; // found wrong file name
			}
		}

		// all files matched -> return cached map
		return cachedMap;
	}

	@Override
	public MapBasedMovement replicate() {
		return new MapBasedMovement(this);
	}

	public Coord getLastLocation() {
		if (lastMapNode != null) {
			return lastMapNode.getLocation();
		} else {
			return null;
		}
	}

	public void setLocation(Coord lastWaypoint) {
		// TODO: This should be optimized
		MapNode nearest = null;
		double minDistance = Double.MAX_VALUE;
		Iterator<MapNode> iterator = getMap().getNodes().iterator();
		while (iterator.hasNext()) {
			MapNode temp = iterator.next();
			double distance = temp.getLocation().distance(lastWaypoint);
			if (distance < minDistance) {
				minDistance = distance;
				nearest = temp;
			}
		}
		lastMapNode = nearest;
	}

	public boolean isReady() {
		return true;
	}

}
