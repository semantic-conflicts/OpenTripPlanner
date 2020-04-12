package org.opentripplanner.routing.spt;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.opentripplanner.routing.core.State;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.graph.Vertex;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import com.google.common.collect.HashMultiset;
import com.google.common.collect.Multiset;
public class MultiShortestPathTree extends AbstractShortestPathTree {
  public static double WALK_DIST_EPSILON=0.05;
  public static double WEIGHT_EPSILON=0.02;
  public static int WEIGHT_DIFF_MARGIN=30;
  public static double TIME_EPSILON=0.02;
  public static int TIME_DIFF_MARGIN=30;
  public static Logger LOG=LoggerFactory.getLogger(MultiShortestPathTree.class);
  public Map<Vertex,List<State>> stateSets;
  public void dump(){
    Multiset<Integer> histogram=HashMultiset.create();
    int statesCount=0;
    int maxSize=0;
    for (    Map.Entry<Vertex,List<State>> kv : stateSets.entrySet()) {
      List<State> states=kv.getValue();
      int size=states.size();
      histogram.add(size);
      statesCount+=size;
      if (size > maxSize) {
        maxSize=size;
      }
    }
    LOG.info("SPT: vertices: " + stateSets.size() + " states: total: "+ statesCount+ " per vertex max: "+ maxSize+ " avg: "+ (statesCount * 1.0 / stateSets.size()));
    List<Integer> nStates=new ArrayList<Integer>(histogram.elementSet());
    Collections.sort(nStates);
    for (    Integer nState : nStates) {
      LOG.info(nState + " states: " + histogram.count(nState)+ " vertices.");
    }
  }
  public MultiShortestPathTree(  RoutingRequest options){
    super(options);
    stateSets=new IdentityHashMap<Vertex,List<State>>();
  }
  public Set<Vertex> getVertices(){
    return stateSets.keySet();
  }
  /** 
 * {@link ShortestPathTree} Interface
 */
  @Override public boolean add(  State newState){
    Vertex vertex=newState.getVertex();
    List<State> states=stateSets.get(vertex);
    if (states == null) {
      states=new ArrayList<State>();
      stateSets.put(vertex,states);
      states.add(newState);
      return true;
    }
    Iterator<State> it=states.iterator();
    while (it.hasNext()) {
      State oldState=it.next();
      if (dominates(oldState,newState))       return false;
      if (dominates(newState,oldState))       it.remove();
    }
    states.add(newState);
    return true;
  }
  public static boolean dominates(  State thisState,  State other){
    if (other.weight == 0) {
      return false;
    }
    if (thisState.isBikeRenting() != other.isBikeRenting())     return false;
    if (thisState.isCarParked() != other.isCarParked())     return false;
    if (thisState.isBikeParked() != other.isBikeParked())     return false;
    if (thisState.backEdge != other.getBackEdge() && ((thisState.backEdge instanceof StreetEdge) && (!((StreetEdge)thisState.backEdge).getTurnRestrictions().isEmpty())))     return false;
    if (thisState.routeSequenceSubset(other)) {
      return thisState.weight <= other.weight && thisState.getElapsedTimeSeconds() <= other.getElapsedTimeSeconds();
    }
    boolean walkDistanceIsHopeful=thisState.walkDistance / other.getWalkDistance() < 1 + WALK_DIST_EPSILON;
    double weightRatio=thisState.weight / other.weight;
    boolean weightIsHopeful=(weightRatio < 1 + WEIGHT_EPSILON && thisState.weight - other.weight < WEIGHT_DIFF_MARGIN);
    double t1=(double)thisState.getElapsedTimeSeconds();
    double t2=(double)other.getElapsedTimeSeconds();
    double timeRatio=t1 / t2;
    boolean timeIsHopeful=(timeRatio < 1 + TIME_EPSILON) && (t1 - t2 <= TIME_DIFF_MARGIN);
    return walkDistanceIsHopeful && weightIsHopeful && timeIsHopeful;
  }
  @Override public State getState(  Vertex dest){
    Collection<State> states=stateSets.get(dest);
    if (states == null)     return null;
    State ret=null;
    for (    State s : states) {
      if ((ret == null || s.betterThan(ret)) && s.isFinal() && s.allPathParsersAccept()) {
        ret=s;
      }
    }
    return ret;
  }
  @Override public List<State> getStates(  Vertex dest){
    return stateSets.get(dest);
  }
  @Override public int getVertexCount(){
    return stateSets.keySet().size();
  }
  /** 
 * Check that a state coming out of the queue is still in the Pareto-optimal set for this vertex,  which indicates that it has not been ruled out as a state on an optimal path. Many shortest  path algorithms will decrease the key of an entry in the priority queue when it is updated, or remove it when it is dominated. When the Fibonacci heap was replaced with a binary heap, the decrease-key operation was  removed for the same reason: both improve theoretical run time complexity, at the cost of  high constant factors and more complex code. So there can be dominated (useless) states in the queue. When they come out we want to  ignore them rather than spend time branching out from them.
 */
  @Override public boolean visit(  State state){
    boolean ret=false;
    for (    State s : stateSets.get(state.getVertex())) {
      if (s == state) {
        ret=true;
        break;
      }
    }
    return ret;
  }
  public String toString(){
    return "MultiSPT(" + this.stateSets.size() + " vertices)";
  }
  @Override public Collection<State> getAllStates(){
    ArrayList<State> allStates=new ArrayList<State>();
    for (    List<State> stateSet : stateSets.values()) {
      allStates.addAll(stateSet);
    }
    return allStates;
  }
  public MultiShortestPathTree(){
  }
}
