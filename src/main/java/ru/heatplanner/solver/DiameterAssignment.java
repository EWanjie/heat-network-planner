package ru.heatplanner.solver;

import java.util.*;

/** Pareto dynamic programming over constant-flow chains of a rooted forest. */
final class DiameterAssignment {
    static final class State {
        final int diameter;
        final double suffix,cost;
        final Map<String,Integer> assignment;
        State(int d,double suffix,double cost,Map<String,Integer>a){diameter=d;this.suffix=suffix;this.cost=cost;assignment=a;}
    }
    private final PlanningContext context;
    DiameterAssignment(PlanningContext context){this.context=context;}
    boolean assign(NetworkCandidate network) {
        Map<String,Integer> assignment=new HashMap<>();
        for(NetworkCandidate.Node root:network.nodes.values())if(root.root!=null) {
            List<NetworkCandidate.Edge> children=network.children(root.key);
            if(children.isEmpty())return false;
            List<List<State>> choices=new ArrayList<>();
            for(NetworkCandidate.Edge e:children){computeFlow(network,e,new HashSet<>());List<State>s=states(network,e);if(s.isEmpty())return false;choices.add(s);}
            // Root price is coupled to the largest incoming/new DN (or fixed existing tie-in price).
            State best=null;
            for(int d=context.existingDiameter(root.point);d<PipeCatalog.DN.length;d++) {
                double cost=root.root.chamber==null?PipeCatalog.chamber(d):5_000_000.0*children.size();
                Map<String,Integer> picked=new HashMap<>();boolean ok=true;
                for(List<State> states:choices) {
                    final int maximum=d;
                    State s=states.stream().filter(x->x.diameter<=maximum).min(Comparator.comparingDouble((State x)->x.cost).thenComparingInt(x->x.diameter)).orElse(null);
                    if(s==null){ok=false;break;}cost+=s.cost;picked.putAll(s.assignment);
                }
                if(ok&&(best==null||cost<best.cost-1e-5))best=new State(d,0,cost,picked);
            }
            if(best==null)return false;assignment.putAll(best.assignment);
        }
        for(NetworkCandidate.Edge e:network.edges){Integer d=assignment.get(e.key());if(d==null)return false;e.diameter=d;}return true;
    }
    private double computeFlow(NetworkCandidate n,NetworkCandidate.Edge edge,Set<String>seen) {
        if(!seen.add(edge.to))throw new IllegalArgumentException("Цикл в дереве");
        NetworkCandidate.Node node=n.nodes.get(edge.to);double flow=node.target==null?0:node.target.flow();
        for(NetworkCandidate.Edge c:n.children(node.key))flow+=computeFlow(n,c,seen);
        edge.flow=flow;return flow;
    }
    List<State> states(NetworkCandidate n,NetworkCandidate.Edge edge) {
        List<NetworkCandidate.Edge> children=n.children(edge.to);
        List<List<State>> choices=new ArrayList<>();for(NetworkCandidate.Edge c:children)choices.add(states(n,c));
        List<State> result=new ArrayList<>();
        for(int d=0;d<PipeCatalog.DN.length;d++) {
            if(PipeCatalog.CAPACITY[d]<edge.flow||PipeCatalog.MAX_LENGTH[d]<edge.route.getLength())continue;
            Map<String,Integer> own=new HashMap<>();own.put(edge.key(),d);
            double cost=edge.weightedLength*PipeCatalog.PRICE[d]+(children.size()>1?PipeCatalog.chamber(d):0);
            List<State> combinations=new ArrayList<>(List.of(new State(d,edge.route.getLength(),cost,own)));
            for(List<State> childStates:choices) {
                List<State> next=new ArrayList<>();
                for(State parent:combinations)for(State child:childStates) {
                    if(child.diameter>d)continue;
                    // A degree-two technical node must not reset or change constant-flow diameter.
                    if(children.size()==1&&child.diameter!=d)continue;
                    double suffix=Math.max(parent.suffix,edge.route.getLength()+(child.diameter==d?child.suffix:0));
                    if(suffix>PipeCatalog.MAX_LENGTH[d]+MetricGeometry.EPS)continue;
                    Map<String,Integer> a=new HashMap<>(parent.assignment);a.putAll(child.assignment);
                    retain(next,new State(d,suffix,parent.cost+child.cost,a));
                }combinations=next;
            }
            result.addAll(combinations);
        }return result;
    }
    private void retain(List<State> states,State candidate) {
        if(states.stream().anyMatch(s->s.cost<=candidate.cost+1e-5&&s.suffix<=candidate.suffix+1e-7))return;
        states.removeIf(s->candidate.cost<=s.cost+1e-5&&candidate.suffix<=s.suffix+1e-7);states.add(candidate);
    }
}
