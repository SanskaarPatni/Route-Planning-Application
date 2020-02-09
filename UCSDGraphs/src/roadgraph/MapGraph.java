/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.PriorityQueue;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private HashMap<GeographicPoint,MapNode>mapGraph;
	private HashSet<MapEdge>edges;
	
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		mapGraph=new HashMap<GeographicPoint,MapNode>();
		edges = new HashSet<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return mapGraph.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		Set<GeographicPoint>verticess=mapGraph.keySet();
		return verticess;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		if(mapGraph.containsKey(location) || location==null)
		{
			return false;	
		}
		// TODO: Implement this method in WEEK 3
		MapNode mapNode=new MapNode(location);
		mapGraph.put(location,mapNode);
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		if(mapGraph.containsKey(from) && mapGraph.containsKey(to)) {
			MapNode start=mapGraph.get(from);
			MapNode end=mapGraph.get(to);
			MapEdge mapEdge=new MapEdge(start,end,roadName,roadType,length);
			edges.add(mapEdge);
			start.addEdge(mapEdge);
			
		}
		else if(length<0) {
			throw new IllegalArgumentException();
		}
		else {
			throw new IllegalArgumentException();
		}

		//TODO: Implement this method in WEEK 3
		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		//Time Complexity of O(E+V)
		// Hook for visualization.  See writeup.
		//If any one or both are null
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		
		MapNode startNode = mapGraph.get(start);
		MapNode endNode = mapGraph.get(goal);

		HashSet<MapNode> visited = new HashSet<MapNode>();
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		
		toExplore.add(startNode);
		
		boolean found = false;
		
		visited.add(startNode);
		
		MapNode curr=null;
		
		while (!toExplore.isEmpty()) {
			curr = toExplore.remove();
			nodeSearched.accept(curr.getLocation());
			//Just for checking
			System.out.println(curr.getLocation());
			
			if (curr.equals(endNode)) {
				found = true;
				break;
			}
			
			List<MapNode> neighbors = getNeighbors(curr);
			ListIterator<MapNode> it = neighbors.listIterator(neighbors.size());
			
			while (it.hasPrevious()) {
				MapNode next = it.previous();
				if (!visited.contains(next)) {
					visited.add(next);
					parentMap.put(next, curr);
					toExplore.add(next);
				}
			}
		}

		if (!found) {
			System.out.println("No path exists");
			return new ArrayList<GeographicPoint>();
		}
		// reconstruct the path
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode CURR = endNode;
		while (CURR != startNode) {
			path.addFirst(CURR.getLocation());
			CURR = parentMap.get(CURR);
		}
		path.addFirst(start);
		return path;

	}
	
	List<MapNode>getNeighbors(MapNode start){
		List<MapNode>getNeighborsList=new ArrayList<MapNode>();
		Set<MapEdge>vertexEdges=start.getEdges();
		if(vertexEdges!=null) {
		for(MapEdge mE:vertexEdges) {
			getNeighborsList.add(mE.getEndNode());
		}
		}
		
		return getNeighborsList;
	}
	
	void printGraph() 
	{
		System.out.println("Graph is:\n");
		
			System.out.println("Number of nodes "+getNumVertices());
			System.out.println("\nNumber of edges "+getNumEdges());
			for(GeographicPoint g:mapGraph.keySet()) {	
				System.out.println("\nNumber of edges "+mapGraph.get(g).getEdgesSize());
			}
	}
	
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		 //O(E)*O(logE)+O(V)
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		MapNode startNode = mapGraph.get(start);
		MapNode endNode =mapGraph.get(goal);
		
		HashSet<MapNode> visited = new HashSet<MapNode>();
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		
		startNode.setDistance(0.0);
		int count=0;
		MapNode curr=null;
		toExplore.add(startNode);
		while(!toExplore.isEmpty())
		{
			
			curr = toExplore.poll();
			count++;
			//System.out.println(curr.getLocation());
			
			if(visited.contains(curr)==false)
			{
				visited.add(curr);
				nodeSearched.accept(curr.getLocation());
				if(curr.equals(endNode)) {
					break;
				}
				for (MapEdge edge : curr.getEdges()){
					if(!visited.contains(edge.getEndNode())){
						double currentDistance = edge.getDistance() + curr.getDist();
						if (currentDistance < edge.getEndNode().getDist()){
							parentMap.put(edge.getEndNode(),curr);
							edge.getEndNode().setDistance(currentDistance);
							toExplore.add(edge.getEndNode());
						}
					}
				}
				
			}
		}
		System.out.println("Visited: " + visited.size());
	    System.out.println(count);
		
		if (!endNode.equals(curr)) {
			System.out.println("No path exists");
			return new ArrayList<GeographicPoint>();
		}
		
		// reconstruct the path
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode CURR = endNode;
		while (CURR != startNode) {
			path.addFirst(CURR.getLocation());
			CURR = parentMap.get(CURR);
		}
		path.addFirst(start);
		return path;
	}

	
	

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		MapNode startNode = mapGraph.get(start);
		MapNode endNode = mapGraph.get(goal);
		
		// check for null values
		if (startNode == null || endNode == null){
			return null;
		}
		int count = 0;
		// starting by adding the start node to the queue and to the visited
		startNode.setDistance(0);
		startNode.setActualDistance(0);
		toExplore.add(startNode);
		MapNode curr=null;
		while(!toExplore.isEmpty())
		{
			curr=toExplore.poll();
			count++;
			System.out.println(curr.getLocation());
		
			nodeSearched.accept(curr.getLocation());
			if(curr.equals(endNode)) {
				visited.add(curr);
				break;
			}
			if(!visited.contains(curr)) {
				visited.add(curr);
				
				for (MapEdge edge : curr.getEdges()){
					if(!visited.contains(edge.getEndNode())){
						double currentDistance = edge.getDistance() + curr.getActualDistance();
						// takes the current distance and adds the distance of the edge's end node location to the final endNode location.
						double predictDistance = currentDistance + (edge.getEndNode().getLocation()).distance(endNode.getLocation());
						System.out.println(predictDistance);
						System.out.println(edge.getEndNode().getDist());
						if (predictDistance < edge.getEndNode().getDist()){
							//Double.POSITIVE_INFINITY
							parentMap.put(edge.getEndNode(), curr);
							edge.getEndNode().setActualDistance(currentDistance);
							edge.getEndNode().setDistance(predictDistance);
							toExplore.add(edge.getEndNode());
						}
					}
				}
			}
			}
		System.out.println("\nVisited: " + visited.size());
		System.out.println("Nodes visited so far in A*: " + count);
		if (!endNode.equals(curr)) {
			System.out.println("No path exists");
			return new ArrayList<GeographicPoint>();
		}
		
		// reconstruct the path
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode CURR = endNode;
		while (CURR != startNode) {
			path.addFirst(CURR.getLocation());
			CURR = parentMap.get(CURR);
		}
		path.addFirst(start);
		return path;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		/*MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		firstMap.printGraph();
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		List<GeographicPoint>bl= firstMap.bfs(testStart,testEnd);
		System.out.println(bl);*/
		 
//		MapGraph simpleTestMap = new MapGraph();
//		 GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
//		 GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
//		 GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
//			
//	 	 /*System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
//		 List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
//         System.out.println(testroute);*/
//         List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
//         System.out.println(testroute2);
//		 
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		//List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		/*MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);*/
		
		/*MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		// A slightly more complex test using real data
		GeographicPoint testStart = new GeographicPoint(32.8674388, -117.2190213);
		GeographicPoint testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		List<GeographicPoint> testroute = testMap.dijkstra(testStart,testEnd);
		List<GeographicPoint>testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
        /* MapGraph simpleTestMap = new MapGraph();
 		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
 		
 		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
 		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
 		
 		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
 		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
 		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
 		
 		
 		MapGraph testMap = new MapGraph();
 		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
 		
 		// A very simple test using real data
 		testStart = new GeographicPoint(32.869423, -117.220917);
 		testEnd = new GeographicPoint(32.869255, -117.216927);
 		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
 		testroute = testMap.dijkstra(testStart,testEnd);
 		testroute2 = testMap.aStarSearch(testStart,testEnd);
 		
 		
 		// A slightly more complex test using real data
 		testStart = new GeographicPoint(32.8674388, -117.2190213);
 		testEnd = new GeographicPoint(32.8697828, -117.2244506);
 		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
 		testroute = testMap.dijkstra(testStart,testEnd);
 		testroute2 = testMap.aStarSearch(testStart,testEnd);*/
		
	}
	
}
