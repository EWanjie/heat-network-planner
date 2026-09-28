package ru.heatplanner.solver;

import org.locationtech.jts.geom.*;
import java.util.*;
import static ru.heatplanner.solver.MetricGeometry.*;

/** Rebuilds topology/flows/lengths/costs from the candidate instead of trusting search labels. */
final class NetworkValidator {
    final PlanningContext context;
    final GeometryRules rules;
    String failure="";
    NetworkValidator(PlanningContext c){context=c;rules=new GeometryRules(c);}
    boolean fail(String why){failure=why;return false;}
    boolean validate(NetworkCandidate n) {
        n.valid=false;
        if(n.edges.isEmpty()||context.invalidInput)return fail("EMPTY_OR_INVALID_INPUT");
        Map<String,List<NetworkCandidate.Edge>> out=new HashMap<>();Map<String,Integer> indegree=new HashMap<>();
        Set<String> edgeKeys=new HashSet<>();
        for(NetworkCandidate.Edge e:n.edges) {
            if(!edgeKeys.add(e.key())||!n.nodes.containsKey(e.from)||!n.nodes.containsKey(e.to))return fail("DUPLICATE_OR_MISSING_NODE");
            indegree.merge(e.to,1,Integer::sum);out.computeIfAbsent(e.from,k->new ArrayList<>()).add(e);
            Coordinate[] cs=e.route.getCoordinates();
            if(cs[0].distance(n.nodes.get(e.from).point)>EPS||cs[cs.length-1].distance(n.nodes.get(e.to).point)>EPS)return fail("GEOMETRY_ENDPOINT");
        }
        for(NetworkCandidate.Node node:n.nodes.values()) {
            int incoming=indegree.getOrDefault(node.key,0),children=out.getOrDefault(node.key,List.of()).size();
            if(incoming!=(node.root==null?1:0))return fail("ROOT_OR_CYCLE");
            if(node.target!=null&&children!=0)return fail("TARGET_NOT_TERMINAL");
            int degree=incoming+children+(node.root==null?0:context.existingDegree(node.point));
            if(degree>4)return fail("CHAMBER_DEGREE");
            if(node.root!=null&&node.root.chamber==null)for(PlanningContext.Feature c:context.chambers)
                if(c.geometry.getCoordinate().distance(node.point)<=10+EPS&&context.existingDegree(c.geometry.getCoordinate())>0) {
                    int used=n.nodes.values().stream().filter(x->x.root!=null&&x.root.chamber!=null&&x.root.chamber.key.equals(c.key)).mapToInt(x->n.children(x.key).size()).sum();
                    if(context.existingDegree(c.geometry.getCoordinate())+used<4)return fail("CAMERA_WITHIN_10M");
                }
        }
        Set<String> visited=new HashSet<>();
        try {for(NetworkCandidate.Node root:n.nodes.values())if(root.root!=null)walk(root.key,n,out,visited);}catch(IllegalArgumentException e){return fail(e.getMessage());}
        if(visited.size()!=n.nodes.size())return fail("UNREACHABLE_COMPONENT");
        Set<String> connected=new HashSet<>();for(NetworkCandidate.Node node:n.nodes.values())if(node.target!=null)connected.add(node.target.key);
        for(PlanningContext.Feature target:context.targets)if(connected.contains(target.key)==n.pending.contains(target.key))return fail("TARGET_ACCOUNTING");
        for(int i=0;i<n.edges.size();i++)for(int j=i+1;j<n.edges.size();j++) {
            NetworkCandidate.Edge a=n.edges.get(i),b=n.edges.get(j);Geometry hit=a.route.intersection(b.route);
            if(hit.isEmpty())continue;if(hit.getDimension()>0)return fail("OVERLAPPING_NEW_LINES");
            Set<String> common=new HashSet<>(List.of(a.from,a.to));common.retainAll(List.of(b.from,b.to));
            for(Coordinate p:hit.getCoordinates())if(common.stream().noneMatch(k->n.nodes.get(k).point.distance(p)<EPS))return fail("CROSSING_WITHOUT_NODE");
        }
        n.cost=0;n.length=0;n.chamberCost=0;n.tieInCost=0;n.tieIns=0;n.penalty=0;
        for(NetworkCandidate.Edge e:n.edges) {
            NetworkCandidate.Node from=n.nodes.get(e.from),to=n.nodes.get(e.to);
            GeometryRules.Check check=rules.check(e.route,e.diameter,to.target,from.root);
            if(!check.valid)return fail(check.reason);
            if(context.coverage!=null&&!context.coverage.covers(e.route.buffer(12))) {
                context.needCoverage(e.route);return fail("OSM_COVERAGE_INCOMPLETE");
            }
            e.weightedLength=check.weightedLength;
            n.cost+=check.weightedLength*PipeCatalog.PRICE[e.diameter];n.length+=e.route.getLength();
        }
        for(NetworkCandidate.Node node:n.nodes.values()) {
            List<NetworkCandidate.Edge> children=out.getOrDefault(node.key,List.of());
            if(node.root!=null&&node.root.chamber!=null){n.tieIns+=children.size();n.tieInCost+=5_000_000.0*children.size();}
            else if(node.root!=null||children.size()>1) {
                int d=node.root==null?0:context.existingDiameter(node.point);
                for(NetworkCandidate.Edge e:n.edges)if(e.from.equals(node.key)||e.to.equals(node.key))d=Math.max(d,e.diameter);
                n.chamberCost+=PipeCatalog.chamber(d);
            }
        }
        for(PlanningContext.Feature t:context.targets)if(n.pending.contains(t.key))n.penalty+=100_000_000+500_000*t.flow();
        n.cost+=n.chamberCost+n.tieInCost;n.score=PipeCatalog.score(n.cost+n.penalty,n.length);n.valid=true;return true;
    }
    private double walk(String key,NetworkCandidate n,Map<String,List<NetworkCandidate.Edge>>out,Set<String>visited) {
        if(!visited.add(key))throw new IllegalArgumentException("CYCLE");
        NetworkCandidate.Node node=n.nodes.get(key);double flow=node.target==null?0:node.target.flow();
        List<NetworkCandidate.Edge> children=out.getOrDefault(key,List.of());
        for(NetworkCandidate.Edge e:children) {
            double actual=walk(e.to,n,out,visited);flow+=actual;
            if(Math.abs(actual-e.flow)>1e-6||actual>PipeCatalog.CAPACITY[e.diameter]+EPS)throw new IllegalArgumentException("FLOW_OR_CAPACITY");
            if(longest(e,n)>PipeCatalog.MAX_LENGTH[e.diameter]+EPS)throw new IllegalArgumentException("CONTINUOUS_LENGTH");
            NetworkCandidate.Edge incoming=n.incoming(key);
            if(incoming!=null&&(incoming.diameter<e.diameter||(children.size()==1&&incoming.diameter!=e.diameter)))throw new IllegalArgumentException("DIAMETER_MONOTONICITY");
        }return flow;
    }
    private double longest(NetworkCandidate.Edge edge,NetworkCandidate n) {
        double child=0;for(NetworkCandidate.Edge e:n.children(edge.to))if(e.diameter==edge.diameter)child=Math.max(child,longest(e,n));
        return edge.route.getLength()+child;
    }
}
