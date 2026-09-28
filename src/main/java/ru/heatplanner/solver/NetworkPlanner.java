package ru.heatplanner.solver;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.locationtech.jts.geom.*;
import org.locationtech.jts.linearref.LengthIndexedLine;
import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;
import static ru.heatplanner.solver.MetricGeometry.*;

/** Bounded beam search over forests; all returned variants pass independent validation. */
public final class NetworkPlanner {
    private final ObjectMapper mapper;
    public NetworkPlanner(ObjectMapper mapper){this.mapper=mapper;}
    public Map<String,Object> solveNetwork(JsonNode input,JsonNode roadMetadata,SearchOptions options,BooleanSupplier cancellation) {
        options.validate();
        PlanningContext context=new PlanningContext(input,roadMetadata);
        SearchBudget budget=new SearchBudget(options,cancellation);
        Engine engine=new Engine(context,budget);
        List<NetworkCandidate> pool=new ArrayList<>();
        if(context.pipes.isEmpty())context.diagnostic("NO_EXISTING_NETWORK",null,"Нет существующей сети для подключения");
        if(context.targets.isEmpty())context.diagnostic("NO_TARGETS",null,"Нет точек подключения");
        if(!context.invalidInput&&!context.pipes.isEmpty()) {
            NetworkCandidate seed=engine.seedConnections();
            if(!seed.edges.isEmpty()&&engine.validator.validate(seed))pool.add(seed);
            for(RouteSearch.Profile profile:RouteSearch.Profile.values()) {
                if(budget.exhausted())break;
                List<List<PlanningContext.Feature>> orders=new ArrayList<>();
                List<PlanningContext.Feature> difficult=new ArrayList<>(context.targets);
                difficult.sort(Comparator.<PlanningContext.Feature>comparingDouble(t->context.pipes.stream().mapToDouble(p->p.geometry.distance(t.geometry)).min().orElse(0)).reversed().thenComparing(t->t.key));
                orders.add(difficult);
                List<PlanningContext.Feature> flow=new ArrayList<>(context.targets);flow.sort(Comparator.<PlanningContext.Feature>comparingDouble(PlanningContext.Feature::flow).reversed().thenComparing(t->t.key));orders.add(flow);
                List<PlanningContext.Feature> shuffled=new ArrayList<>(context.targets);Collections.shuffle(shuffled,new Random(options.seed));orders.add(shuffled);
                for(List<PlanningContext.Feature> order:orders) {
                    if(budget.exhausted())break;
                    NetworkCandidate empty=new NetworkCandidate();empty.profile=profile;context.targets.forEach(t->empty.pending.add(t.key));
                    List<NetworkCandidate> beam=new ArrayList<>(List.of(empty));
                    for(PlanningContext.Feature target:order) {
                        if(budget.exhausted())break;
                        List<NetworkCandidate> proposals=new ArrayList<>();
                        for(NetworkCandidate state:beam) {
                            List<NetworkCandidate> attached=engine.attach(state,target);
                            if(attached.isEmpty())proposals.add(state);else proposals.addAll(attached);
                            if(budget.exhausted())break;
                        }
                        beam=engine.retain(proposals,profile);
                    }
                    for(NetworkCandidate state:beam)if(!state.edges.isEmpty()) {
                        // Keep verified incumbents even if later local exploration exhausts the budget.
                        if(engine.validator.validate(state))pool.add(state);
                        List<NetworkCandidate> improved=engine.improve(state);
                        pool.addAll(improved);
                    }
                }
            }
        }
        List<NetworkCandidate> recovered=new ArrayList<>();
        for(NetworkCandidate n:pool) {
            NetworkCandidate best=n;
            for(PlanningContext.Feature target:context.targets)if(best.pending.contains(target.key)&&!budget.exhausted()) {
                List<NetworkCandidate> attached=engine.attach(best,target);
                if(!attached.isEmpty())best=attached.stream().min(Comparator.comparingDouble(x->x.score)).get();
            }recovered.add(best);
        }
        List<NetworkCandidate> selected=engine.select(recovered);
        for(PlanningContext.Feature target:context.targets)if(selected.isEmpty()||selected.stream().anyMatch(n->n.pending.contains(target.key)))
            context.diagnostic(budget.exhausted()?"SEARCH_BUDGET_EXHAUSTED":"NO_ROUTE_FOUND_AFTER_REFINEMENT",target,"Не найден проверенный маршрут; это не доказательство невозможности подключения");
        if(budget.exhausted())context.diagnostic(cancellation.getAsBoolean()?"CANCELLED":"SEARCH_BUDGET_EXHAUSTED",null,"Возвращены только сохранённые проверенные кандидаты");
        List<JsonNode> variants=new ArrayList<>();
        GeoJsonResult writer=new GeoJsonResult(context,mapper);
        ResultValidator outputValidator=new ResultValidator();
        for(NetworkCandidate candidate:selected) {
            JsonNode output=writer.write(candidate,variants.size()+1);
            List<String> errors=outputValidator.validate(context,output);budget.validations++;
            if(errors.isEmpty())variants.add(output);
            else context.diagnostic("OUTPUT_VALIDATION_FAILED",null,String.join("; ",errors));
        }
        Map<String,Object> result=new LinkedHashMap<>();result.put("variants",variants);result.put("diagnostics",Map.of("issues",context.diagnostics,"preliminaryCandidates",List.of()));
        Map<String,Object> stats=new LinkedHashMap<>();stats.put("elapsedMillis",(System.nanoTime()-budget.started)/1_000_000);
        stats.put("expansions",budget.expansions);stats.put("checkedTransitions",budget.transitions);stats.put("routeCalls",budget.routeCalls);
        stats.put("candidates",budget.candidates);stats.put("validations",budget.validations);stats.put("refinements",budget.refinements);stats.put("seed",options.seed);
        stats.put("budgetExhausted",budget.exhausted());stats.put("method","sparse-visibility-directional-A*-diameter-DP-beam");stats.put("globalOptimumGuaranteed",false);
        result.put("searchStatistics",stats);
        if(!context.requestedCoverage.isNull()) {
            Envelope e=context.requestedCoverage;result.put("requiredOsmBbox",List.of(e.getMinX(),e.getMinY(),e.getMaxX(),e.getMaxY()));
        }
        return result;
    }
    static final class Engine {
        final PlanningContext c;final SearchBudget budget;final RouteSearch routes;final GeometryRules rules;
        final DiameterAssignment diameters;final NetworkValidator validator;
        Engine(PlanningContext c,SearchBudget budget){this.c=c;this.budget=budget;routes=new RouteSearch(c,budget);rules=routes.rules;diameters=new DiameterAssignment(c);validator=new NetworkValidator(c);}
        NetworkCandidate seedConnections() {
            NetworkCandidate state=new NetworkCandidate();state.profile=RouteSearch.Profile.BALANCED;
            for(PlanningContext.Feature target:c.targets)state.pending.add(target.key);
            List<PlanningContext.Feature> targets=new ArrayList<>(c.targets);
            targets.sort(Comparator.comparing(t->t.key));
            Map<String,List<PlanningContext.Root>> roots=new LinkedHashMap<>();
            for(PlanningContext.Feature target:targets)roots.put(target.key,c.roots(target.geometry.getCoordinate(),budget.options,0));
            long previousLimit=budget.scopeExpansions,previousDeadline=budget.scopeDeadline;
            budget.scopeExpansions=Math.min(previousLimit,budget.options.totalSearchBudget*3/4);
            budget.scopeDeadline=Math.min(previousDeadline,budget.started+budget.options.maxTimeMillis*3/4*1_000_000);
            try {
                // Try each target once before spending another search on a different tie-in.
                for(int rootIndex=0;rootIndex<budget.options.initialTieInCandidateCount&&!budget.exhausted();rootIndex++) {
                    for(PlanningContext.Feature target:targets) {
                        if(budget.exhausted())break;
                        if(!state.pending.contains(target.key)||rootIndex>=roots.get(target.key).size())continue;
                        PlanningContext.Root root=roots.get(target.key).get(rootIndex);
                        if(c.existingDegree(root.point)+state.children(root.key()).size()>=4)continue;
                        Coordinate end=target.geometry.getCoordinate();int d=PipeCatalog.minimum(target.flow(),end.distance(root.point));
                        if(d<0||!routes.terminalPossible(target,d))continue;
                        LineString route=routes.findAtLevel(root.point,end,d,PipeCatalog.MAX_LENGTH[d],target,root,state.profile,0);
                        if(route==null)continue;
                        NetworkCandidate next=state.copy();String to="target:"+target.key;
                        next.nodes.putIfAbsent(root.key(),new NetworkCandidate.Node(root.key(),root.point,root,null));
                        next.nodes.put(to,new NetworkCandidate.Node(to,end,null,target));
                        next.edges.add(new NetworkCandidate.Edge(root.key(),to,route,d));next.pending.remove(target.key);
                        if(evaluate(next))state=next;
                        else {
                            NetworkCandidate joined=joinExistingRoute(state,target,route,d);
                            if(joined!=null)state=joined;
                        }
                    }
                }
                return state;
            } finally {budget.scopeExpansions=previousLimit;budget.scopeDeadline=previousDeadline;}
        }
        NetworkCandidate joinExistingRoute(NetworkCandidate state,PlanningContext.Feature target,LineString route,int d) {
            LengthIndexedLine index=new LengthIndexedLine(route);
            double last=-1;NetworkCandidate.Edge touched=null;Coordinate contact=null;
            for(NetworkCandidate.Edge edge:state.edges) {
                Geometry intersection=route.intersection(edge.route);
                for(Coordinate p:intersection.getCoordinates()) {
                    double at=index.project(p);
                    if(at>last&&at<route.getLength()-EPS){last=at;touched=edge;contact=p;}
                }
            }
            if(touched==null)return null;
            NetworkCandidate next=state.copy();
            String junction=split(next,touched.key(),new LengthIndexedLine(touched.route).project(contact));
            if(junction==null||next.children(junction).size()>=3)return null;
            String to="target:"+target.key;
            next.nodes.put(to,new NetworkCandidate.Node(to,target.geometry.getCoordinate(),null,target));
            next.edges.add(new NetworkCandidate.Edge(junction,to,(LineString)index.extractLine(last,route.getLength()),d));
            next.pending.remove(target.key);
            return evaluate(next)?next:null;
        }
        List<NetworkCandidate> attach(NetworkCandidate state,PlanningContext.Feature target) {
            long previousLimit=budget.scopeExpansions,previousDeadline=budget.scopeDeadline;
            int shares=Math.max(1,c.targets.size()*3);
            budget.scopeExpansions=Math.min(previousLimit,budget.expansions+Math.max(100,budget.options.totalSearchBudget/shares));
            budget.scopeDeadline=Math.min(previousDeadline,System.nanoTime()+Math.max(200,budget.options.maxTimeMillis/shares)*1_000_000);
            try{return attachWithinBudget(state,target);}finally{budget.scopeExpansions=previousLimit;budget.scopeDeadline=previousDeadline;}
        }
        List<NetworkCandidate> attachWithinBudget(NetworkCandidate state,PlanningContext.Feature target) {
            List<NetworkCandidate> result=new ArrayList<>();
            int minimum=PipeCatalog.minimum(target.flow(),0);
            if(minimum<0){c.diagnostic("NO_SUPPORTED_DIAMETER_FOR_CANDIDATE",target,"Расход превышает справочник ДУ");return result;}
            if(!routes.terminalPossible(target,minimum)) {
                c.diagnostic("TERMINAL_APPROACH_BLOCKED",target,"Прямые подходы через ближайшую границу ОКС пересекают габариты других непроходимых объектов");return result;
            }
            Coordinate end=target.geometry.getCoordinate();
            // Separate connections are seeds, not the only topology considered.
            for(int level=0;level<budget.options.maxGraphRefinements&&!budget.exhausted();level++) {
                for(PlanningContext.Root root:c.roots(end,budget.options,level)) {
                    if(budget.exhausted())break;
                    String rootKey=root.key();int used=state.children(rootKey).size();
                    if(c.existingDegree(root.point)+used>=4)continue;
                    int first=PipeCatalog.minimum(target.flow(),end.distance(root.point));
                    if(first<0)continue;
                    // Interleave diameter widening with tie-in widening instead of spending the whole budget at one root.
                    for(int d=Math.min(PipeCatalog.DN.length-1,first+level);d<=Math.min(PipeCatalog.DN.length-1,first+level)&&!budget.exhausted();d++) {
                        if(PipeCatalog.MAX_LENGTH[d]<end.distance(root.point))continue;
                        LineString route=routes.find(root.point,end,d,PipeCatalog.MAX_LENGTH[d],target,root,state.profile,null);
                        if(route==null)continue;
                        NetworkCandidate n=state.copy();n.nodes.putIfAbsent(rootKey,new NetworkCandidate.Node(rootKey,root.point,root,null));
                        String to="target:"+target.key;n.nodes.put(to,new NetworkCandidate.Node(to,end,null,target));
                        n.edges.add(new NetworkCandidate.Edge(rootKey,to,route,d));n.pending.remove(target.key);
                        if(evaluate(n))result.add(n);break;
                    }
                }
                if(!result.isEmpty())break;
            }
            // Attach to existing branches, including projection/split points and junctions.
            List<NetworkCandidate.Edge> near=new ArrayList<>(state.edges);
            near.sort(Comparator.comparingDouble(e->e.route.distance(target.geometry)));
            for(NetworkCandidate.Edge e:near.subList(0,Math.min(near.size(),budget.options.maxTieInCandidateCount))) {
                if(budget.exhausted())break;
                LengthIndexedLine ref=new LengthIndexedLine(e.route);double at=ref.project(end);
                for(double position:new double[]{at,e.route.getLength()/2}) {
                    if(budget.exhausted())break;
                    NetworkCandidate n=state.copy();String junction=split(n,e.key(),position);
                    if(junction==null||n.children(junction).size()>=3)continue;
                    Coordinate start=n.nodes.get(junction).point;
                    LineString route=routes.find(start,end,minimum,PipeCatalog.MAX_LENGTH[minimum],target,n.nodes.get(junction).root,state.profile,null);
                    if(route==null)continue;
                    String to="target:"+target.key;n.nodes.put(to,new NetworkCandidate.Node(to,end,null,target));
                    n.edges.add(new NetworkCandidate.Edge(junction,to,route,minimum));n.pending.remove(target.key);
                    if(evaluate(n))result.add(n);
                }
            }
            return retain(result,state.profile);
        }
        String split(NetworkCandidate n,String edgeKey,double at) {
            NetworkCandidate.Edge e=n.edges.stream().filter(x->x.key().equals(edgeKey)).findFirst().orElse(null);if(e==null)return null;
            if(at<.01)return n.nodes.get(e.from).target==null?e.from:null;
            if(at>e.route.getLength()-.01)return n.nodes.get(e.to).target==null?e.to:null;
            LengthIndexedLine ref=new LengthIndexedLine(e.route);Coordinate p=ref.extractPoint(at);String key="junction:"+MetricGeometry.key(p);
            if(n.nodes.containsKey(key))return key;
            n.nodes.put(key,new NetworkCandidate.Node(key,p,null,null));n.edges.remove(e);
            n.edges.add(new NetworkCandidate.Edge(e.from,key,(LineString)ref.extractLine(0,at),e.diameter));
            n.edges.add(new NetworkCandidate.Edge(key,e.to,(LineString)ref.extractLine(at,e.route.getLength()),e.diameter));return key;
        }
        boolean evaluate(NetworkCandidate n) {
            budget.candidates++;
            Set<String> signatures=new HashSet<>();
            for(int iteration=0;iteration<budget.options.maxRepairIterations;iteration++) {
                if(budget.exhausted())return false;
                if(!signatures.add(n.signature()))return false;
                for(NetworkCandidate.Edge e:n.edges) {
                    GeometryRules.Check check=rules.check(e.route,e.diameter,n.nodes.get(e.to).target,n.nodes.get(e.from).root);
                    // Tariff boundaries depend on geometry; a new DN is then checked/repaired below.
                    if(check.valid)e.weightedLength=check.weightedLength;
                    else if(e.weightedLength==0)e.weightedLength=e.route.getLength();
                }
                if(!diameters.assign(n))return false;
                boolean changed=false;
                for(NetworkCandidate.Edge e:n.edges) {
                    GeometryRules.Check check=rules.check(e.route,e.diameter,n.nodes.get(e.to).target,n.nodes.get(e.from).root);
                    if(check.valid){e.weightedLength=check.weightedLength;continue;}
                    LineString route=routes.find(n.nodes.get(e.from).point,n.nodes.get(e.to).point,e.diameter,PipeCatalog.MAX_LENGTH[e.diameter],n.nodes.get(e.to).target,n.nodes.get(e.from).root,n.profile,null);
                    if(route==null)return false;e.route=route;changed=true;
                }
                if(!changed){budget.validations++;return validator.validate(n);}
            }return false;
        }
        List<NetworkCandidate> retain(List<NetworkCandidate> proposals,RouteSearch.Profile profile) {
            // Never prefer dropping a reachable target because its penalty happens to be cheaper.
            proposals.sort(Comparator.<NetworkCandidate>comparingInt(NetworkCandidate::connected).reversed()
                    .thenComparingDouble(n->RouteSearch.weight(n.cost+n.penalty,n.length,profile)).thenComparing(NetworkCandidate::signature));
            Set<String> seen=new HashSet<>();List<NetworkCandidate> result=new ArrayList<>();
            for(NetworkCandidate n:proposals)if(seen.add(n.signature())){result.add(n);if(result.size()>=budget.options.beamWidth)break;}return result;
        }
        List<NetworkCandidate> improve(NetworkCandidate start) {
            List<NetworkCandidate> beam=new ArrayList<>(List.of(start));
            for(int iteration=0;iteration<budget.options.maxLocalSearchIterations&&!budget.exhausted();iteration++) {
                List<NetworkCandidate> proposals=new ArrayList<>(beam);
                for(NetworkCandidate n:beam) {
                    for(NetworkCandidate.Edge edge:n.edges) {
                        if(budget.exhausted())break;
                        // Reroute: reference corridor penalty affects search only, never final pricing.
                        NetworkCandidate alternate=n.copy();NetworkCandidate.Edge ae=alternate.edges.stream().filter(e->e.key().equals(edge.key())).findFirst().get();
                        LineString path=routes.find(n.nodes.get(edge.from).point,n.nodes.get(edge.to).point,edge.diameter,PipeCatalog.MAX_LENGTH[edge.diameter],n.nodes.get(edge.to).target,n.nodes.get(edge.from).root,n.profile,edge.route.buffer(budget.options.similarityToleranceMeters));
                        if(path!=null){ae.route=path;if(evaluate(alternate))proposals.add(alternate);}
                        // Split/change root: move a complete subtree to a different existing tie-in.
                        for(PlanningContext.Root root:c.roots(n.nodes.get(edge.to).point,budget.options,iteration)) {
                            if(budget.exhausted())break;if(root.key().equals(edge.from))continue;
                            NetworkCandidate moved=detach(n,edge);
                            if(moved.nodes.containsKey(root.key())&&moved.descendants(edge.to).contains(root.key()))continue;
                            moved.nodes.putIfAbsent(root.key(),new NetworkCandidate.Node(root.key(),root.point,root,null));
                            if(reconnect(moved,root.key(),edge.to,edge.diameter))proposals.add(moved);
                        }
                        // Merge/reparent: attach the subtree to a different branch, removing its old root when unused.
                        Set<String> descendants=n.descendants(edge.to);
                        for(NetworkCandidate.Edge destination:n.edges) {
                            if(budget.exhausted())break;
                            if(destination.key().equals(edge.key())||descendants.contains(destination.from)||descendants.contains(destination.to))continue;
                            NetworkCandidate moved=detach(n,edge);
                            double at=new LengthIndexedLine(destination.route).project(n.nodes.get(edge.to).point);
                            String junction=split(moved,destination.key(),at);
                            if(junction!=null&&reconnect(moved,junction,edge.to,edge.diameter))proposals.add(moved);
                        }
                    }
                    // Move junction in free space; reroute all incident chains and resize the tree.
                    for(NetworkCandidate.Node junction:n.nodes.values())if(junction.root==null&&junction.target==null) {
                        for(double[] delta:new double[][]{{5,0},{-5,0},{0,5},{0,-5}}) {
                            if(budget.exhausted())break;
                            NetworkCandidate moved=n.copy();double step=1.0/(1<<iteration);
                            moved.nodes.put(junction.key,new NetworkCandidate.Node(junction.key,new Coordinate(junction.point.x+delta[0]*step,junction.point.y+delta[1]*step),null,null));
                            boolean ok=true;
                            for(NetworkCandidate.Edge e:moved.edges)if(e.from.equals(junction.key)||e.to.equals(junction.key)) {
                                e.route=routes.find(moved.nodes.get(e.from).point,moved.nodes.get(e.to).point,e.diameter,PipeCatalog.MAX_LENGTH[e.diameter],moved.nodes.get(e.to).target,moved.nodes.get(e.from).root,n.profile,null);
                                if(e.route==null){ok=false;break;}
                            }if(ok&&evaluate(moved))proposals.add(moved);
                        }
                    }
                }
                List<NetworkCandidate> next=retain(proposals,start.profile);
                if(next.stream().map(NetworkCandidate::signature).collect(Collectors.toList()).equals(beam.stream().map(NetworkCandidate::signature).collect(Collectors.toList())))break;
                beam=next;
            }return beam;
        }
        boolean reconnect(NetworkCandidate n,String from,String to,int d) {
            if(from.equals(to)||n.children(from).size()>=3)return false;
            LineString path=routes.find(n.nodes.get(from).point,n.nodes.get(to).point,d,PipeCatalog.MAX_LENGTH[d],n.nodes.get(to).target,n.nodes.get(from).root,n.profile,null);
            if(path==null)return false;n.edges.add(new NetworkCandidate.Edge(from,to,path,d));return evaluate(n);
        }
        NetworkCandidate detach(NetworkCandidate original,NetworkCandidate.Edge edge) {
            NetworkCandidate n=original.copy();n.edges.removeIf(e->e.key().equals(edge.key()));
            String node=edge.from;
            while(n.nodes.containsKey(node)&&n.children(node).isEmpty()&&n.nodes.get(node).target==null) {
                NetworkCandidate.Edge incoming=n.incoming(node);n.nodes.remove(node);
                if(incoming==null)break;n.edges.remove(incoming);node=incoming.from;
            }
            // Collapse a non-branching former junction into a single constant-flow chain.
            boolean changed=true;
            while(changed){changed=false;for(NetworkCandidate.Node p:new ArrayList<>(n.nodes.values()))if(p.root==null&&p.target==null&&n.children(p.key).size()==1) {
                NetworkCandidate.Edge in=n.incoming(p.key);if(in==null)continue;
                NetworkCandidate.Edge out=n.children(p.key).get(0);List<Coordinate> coords=new ArrayList<>(Arrays.asList(in.route.getCoordinates()));
                coords.addAll(Arrays.asList(out.route.getCoordinates()).subList(1,out.route.getNumPoints()));
                n.edges.remove(in);n.edges.remove(out);n.nodes.remove(p.key);n.edges.add(new NetworkCandidate.Edge(in.from,out.to,line(coords.toArray(new Coordinate[0])),Math.max(in.diameter,out.diameter)));changed=true;break;
            }}return n;
        }
        List<NetworkCandidate> select(List<NetworkCandidate> pool) {
            pool.removeIf(n->!validator.validate(n));
            int maximum=pool.stream().mapToInt(NetworkCandidate::connected).max().orElse(0);
            pool.removeIf(n->n.connected()<maximum);
            pool.sort(Comparator.comparingDouble((NetworkCandidate n)->n.score).thenComparing(NetworkCandidate::signature));
            List<NetworkCandidate> selected=new ArrayList<>();
            for(NetworkCandidate n:pool) {
                if(!selected.isEmpty()&&n.score>selected.get(0).score*(1+budget.options.maxAlternativeScoreDeterioration))continue;
                if(selected.stream().allMatch(other->different(n,other)))selected.add(n);
                if(selected.size()==3)break;
            }return selected;
        }
        boolean different(NetworkCandidate a,NetworkCandidate b) {
            Set<String> ar=a.nodes.values().stream().filter(n->n.root!=null).map(n->n.root.key()).collect(Collectors.toSet());
            Set<String> br=b.nodes.values().stream().filter(n->n.root!=null).map(n->n.root.key()).collect(Collectors.toSet());
            // Small shifts of a root alone do not create alternatives.
            if(ar.size()!=br.size())return true;
            Set<String> groupsA=groups(a),groupsB=groups(b);if(!groupsA.equals(groupsB))return true;
            Geometry ga=a.geometry().union(),gb=b.geometry().union();
            if(ga.getLength()<EPS||gb.getLength()<EPS)return false;
            double ab=ga.intersection(gb.buffer(budget.options.similarityToleranceMeters)).getLength()/ga.getLength();
            double ba=gb.intersection(ga.buffer(budget.options.similarityToleranceMeters)).getLength()/gb.getLength();
            return Math.min(ab,ba)<budget.options.maxOverlapForDifferentCorridor;
        }
        Set<String> groups(NetworkCandidate n) {
            Set<String> groups=new TreeSet<>();for(NetworkCandidate.Node root:n.nodes.values())if(root.root!=null)
                groups.add(n.descendants(root.key).stream().filter(k->n.nodes.get(k).target!=null).sorted().collect(Collectors.joining("|")));return groups;
        }
    }
}
