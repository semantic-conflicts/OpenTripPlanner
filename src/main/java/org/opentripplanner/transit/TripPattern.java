package org.opentripplanner.transit;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * This is like a transmodel JourneyPattern.
 * All the trips on the same Route that have the same sequence of stops, with the same pickup/dropoff options.
 */
public class TripPattern implements Serializable {

    private static Logger LOG = LoggerFactory.getLogger(TripPattern.class);

    String routeId;
    int directionId = Integer.MIN_VALUE;
    int[] stops;
    // Could be compacted into 2 bits each or a bunch of flags, but is it even worth it?
    PickDropType[] pickups;
    PickDropType[] dropoffs;
    BitSet wheelchairAccessible;
    List<TripSchedule> tripSchedules = new ArrayList<>();

    public TripPattern (TripPatternKey tripPatternKey) {
        int nStops = tripPatternKey.stops.size();
        stops = new int[nStops];
        pickups = new PickDropType[nStops];
        dropoffs = new PickDropType[nStops];
        wheelchairAccessible = new BitSet(nStops);
        for (int s = 0; s < nStops; s++) {
            stops[s] = tripPatternKey.stops.get(s);
            pickups[s] = PickDropType.forGtfsCode(tripPatternKey.pickupTypes.get(s));
            dropoffs[s] = PickDropType.forGtfsCode(tripPatternKey.dropoffTypes.get(s));
        }
        routeId = tripPatternKey.routeId;
    }

    public void addTrip (TripSchedule tripSchedule) {
        tripSchedules.add(tripSchedule);
    }

    public void setOrVerifyDirection (int directionId) {
        if (this.directionId != directionId) {
            if (this.directionId == Integer.MIN_VALUE) {
                this.directionId = directionId;
                LOG.debug("Pattern has route_id {} and direction_id {}", routeId, directionId);
            } else {
                LOG.warn("Trips with different direction IDs are in the same pattern.");
            }
        }
    }

    // Simply write "graph builder annotations" to a log file alongside the graphs.
    // function in gtfs-lib getOrderedStopTimes(string tripId)
    // Test GTFS loading on NL large data set.

}