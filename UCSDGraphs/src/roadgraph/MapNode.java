package roadgraph;


import java.util.HashSet;
import java.util.Set;
import geography.GeographicPoint;


public class MapNode implements Comparable {
	private GeographicPoint location;
	private HashSet<MapEdge>edges;
	private double distance;
	private double actualDistance;
	
	
	
	MapNode(GeographicPoint loc){
		edges=new HashSet<MapEdge>();
		location=loc;
		distance=Double.POSITIVE_INFINITY;
		actualDistance=Double.POSITIVE_INFINITY;
	
	}
	int getEdgesSize() {
		return edges.size();
	}
	
	HashSet<MapEdge>getEdges(){
		return edges;
	}
	GeographicPoint getLocation()
	{
		return location;
	}
	
	
	void addEdge(MapEdge e) {
		edges.add(e);
	}

	Set<MapNode> getNeighbors(){
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for (MapEdge edge : edges){
			neighbors.add(edge.getEndNode());
		}
		return neighbors;
	}
	
	void setDistance(double length) 
	{
		distance=length;
	}
	double getDist()
	{
		return distance;
	}
	public void setActualDistance(double actualDistance) {
	    this.actualDistance = actualDistance;
	}
	
	public double getActualDistance() {
	    return actualDistance;
	}
	public int compareTo(Object o) {
		// convert to map node, may throw exception
		MapNode m = (MapNode)o; 
		return ((Double)this.getDist()).compareTo((Double)m.getDist());
	}

}
