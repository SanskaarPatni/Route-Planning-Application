package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	private MapNode start;
	private MapNode end;
	private String roadName;
	private String roadtype;
	private double length;
	
	MapEdge(MapNode start,MapNode end,String roadName,String roadtype,double length){
		this.start=start;
		this.end=end;
		this.roadName=roadName;
		this.roadtype=roadtype;
		this.length=length;
	}
	
	double getDistance()
	{
		return this.length;
		
	}
	MapNode getEndNode()
	{
		
		return end;
		
	}

}
