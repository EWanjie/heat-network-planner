package ru.heatplanner.solver;

import org.locationtech.jts.geom.*;
import java.util.*;
import java.util.stream.Collectors;

final class NetworkCandidate {
    static final class Node {
        final String key;
        final Coordinate point;
        final PlanningContext.Root root;
        final PlanningContext.Feature target;
        Node(String key,Coordinate point,PlanningContext.Root root,PlanningContext.Feature target){this.key=key;this.point=point;this.root=root;this.target=target;}
    }
    static final class Edge {
        final String from,to;
        LineString route;
        int diameter;
        double flow,weightedLength;
        Edge(String from,String to,LineString route,int diameter){this.from=from;this.to=to;this.route=route;this.diameter=diameter;}
        String key(){return from+">"+to;}
        Edge copy(){Edge e=new Edge(from,to,route,diameter);e.flow=flow;e.weightedLength=weightedLength;return e;}
    }
    final Map<String,Node> nodes=new TreeMap<>();
    final List<Edge> edges=new ArrayList<>();
    final Set<String> pending=new TreeSet<>();
    double cost,length,score,chamberCost,tieInCost,penalty;
    int tieIns;
    RouteSearch.Profile profile=RouteSearch.Profile.BALANCED;
    boolean valid;
    NetworkCandidate copy(){NetworkCandidate n=new NetworkCandidate();n.nodes.putAll(nodes);for(Edge e:edges)n.edges.add(e.copy());n.pending.addAll(pending);n.profile=profile;return n;}
    List<Edge> children(String key){return edges.stream().filter(e->e.from.equals(key)).sorted(Comparator.comparing(Edge::key)).collect(Collectors.toList());}
    Edge incoming(String key){return edges.stream().filter(e->e.to.equals(key)).findFirst().orElse(null);}
    Set<String> descendants(String key){Set<String>s=new TreeSet<>();collect(key,s);return s;}
    private void collect(String key,Set<String>s){if(!s.add(key))return;for(Edge e:children(key))collect(e.to,s);}
    String signature(){
        List<String> bits=new ArrayList<>();
        for(Edge e:edges) {
            StringBuilder b=new StringBuilder(e.from).append('>').append(e.to).append(':').append(e.diameter);
            for(Coordinate c:e.route.getCoordinates())b.append('|').append(MetricGeometry.key(c));bits.add(b.toString());
        }Collections.sort(bits);return String.join(";",bits);
    }
    Geometry geometry(){return MetricGeometry.GF.createMultiLineString(edges.stream().map(e->e.route).toArray(LineString[]::new));}
    int connected(){return (int)nodes.values().stream().filter(n->n.target!=null).count();}
}
