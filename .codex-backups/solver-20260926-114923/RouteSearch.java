package ru.heatplanner.solver;

import org.locationtech.jts.geom.*;
import org.locationtech.jts.operation.distance.DistanceOp;
import org.locationtech.jts.simplify.DouglasPeuckerSimplifier;
import org.locationtech.jts.index.strtree.STRtree;
import java.util.*;
import static ru.heatplanner.solver.MetricGeometry.*;

/** Sparse visibility A*: incoming edge + nondominated (length, objective) labels. */
final class RouteSearch {
    enum Profile { BALANCED, ECONOMIC, COMPACT }
    final PlanningContext context;
    final GeometryRules rules;
    final SearchBudget budget;
    final Map<String,Set<String>> boundaryLinks=new HashMap<>();
    final Set<String> terminalLinks=new HashSet<>();
    RouteSearch(PlanningContext context,SearchBudget budget){this.context=context;this.budget=budget;rules=new GeometryRules(context);}
    boolean terminalPossible(PlanningContext.Feature target,int d) {
        List<PlanningContext.Feature> own=context.ownBuildings(target);if(own.isEmpty())return true;
        Coordinate t=target.geometry.getCoordinate();
        for(PlanningContext.Feature building:own) {
            Geometry boundary=building.geometry.getBoundary();double nearest=boundary.distance(target.geometry);
            for(int part=0;part<boundary.getNumGeometries();part++) {
                Coordinate[] cs=boundary.getGeometryN(part).getCoordinates();
                for(int i=1;i<cs.length;i++) {
                    Coordinate q=new LineSegment(cs[i-1],cs[i]).closestPoint(t);double distance=q.distance(t);
                    if(Math.abs(distance-nearest)>EPS||distance<EPS)continue;
                    double offset=rules.radius(building,d)+.02;
                    Coordinate approach=new Coordinate(q.x+(q.x-t.x)/distance*offset,q.y+(q.y-t.y)/distance*offset);
                    LineString terminal=line(approach,t);boolean blocked=false;
                    for(PlanningContext.Feature f:context.nearby(terminal,12))if(PlanningContext.HARD.contains(f.type)&&!own.contains(f))
                        if(f.geometry.distance(terminal)<rules.radius(f,d)-EPS){blocked=true;break;}
                    if(!blocked)return true;
                }
            }
        }
        return false;
    }
    static final class Label {
        final int vertex,previous;
        final double length,cost,f;
        final Label parent;
        final long sequence;
        boolean stale;
        Label(int v,int prev,double length,double cost,double f,Label parent,long seq){vertex=v;previous=prev;this.length=length;this.cost=cost;this.f=f;this.parent=parent;sequence=seq;}
    }
    LineString find(Coordinate start,Coordinate goal,int d,double lengthBudget,PlanningContext.Feature target,PlanningContext.Root root,Profile profile,Geometry avoid) {
        budget.routeCalls++;
        if(start.distance(goal)<EPS||start.distance(goal)>lengthBudget||budget.exhausted())return null;
        GeometryRules.Check direct=rules.check(line(start,goal),d,target,root);
        if(avoid==null&&direct.valid&&direct.intervals.isEmpty())return line(start,goal);
        for(int refinement=0;refinement<budget.options.maxGraphRefinements;refinement++) {
            if(budget.exhausted())return null;
            budget.refinements++;
            List<Coordinate> vertices=vertices(start,goal,d,target,refinement);
            LineString path=search(vertices,d,lengthBudget,target,root,profile,avoid,refinement);
            if(path!=null)return path;
        }
        return null;
    }
    private List<Coordinate> vertices(Coordinate start,Coordinate goal,int d,PlanningContext.Feature target,int level) {
        boundaryLinks.clear();terminalLinks.clear();
        LinkedHashMap<String,Coordinate> points=new LinkedHashMap<>();
        points.put(key(start),start);points.put(key(goal),goal);
        Envelope area=new Envelope(start,goal);area.expandBy(Math.max(40,start.distance(goal)*(.5+level)));
        for(PlanningContext.Feature obstacle:context.obstacles) {
            if(!area.intersects(obstacle.geometry.getEnvelopeInternal()))continue;
            // Inscribed buffer chords must not cut the actual clearance: inflate conservatively.
            double r=rules.radius(obstacle,d)/Math.cos(Math.PI/32)+.02;
            Geometry buffer=obstacle.geometry.buffer(r,8);
            Geometry boundary=buffer.getBoundary();
            Geometry simplified=boundary;
            for(Coordinate c:simplified.getCoordinates()) if(area.contains(c))points.putIfAbsent(key(c),c);
            for(int component=0;component<boundary.getNumGeometries();component++) {
                Coordinate[] ring=boundary.getGeometryN(component).getCoordinates();
                for(int i=1;i<ring.length;i++)if(area.contains(ring[i-1])&&area.contains(ring[i])) {
                    boundaryLinks.computeIfAbsent(key(ring[i-1]),k->new TreeSet<>()).add(key(ring[i]));
                    boundaryLinks.computeIfAbsent(key(ring[i]),k->new TreeSet<>()).add(key(ring[i-1]));
                }
            }
            if(PlanningContext.SPECIAL.contains(obstacle.type)) {
                for(Coordinate anchor:List.of(start,goal)) {
                    Coordinate q=DistanceOp.nearestPoints(obstacle.geometry,point(anchor))[0];
                    double dx=anchor.x-q.x,dy=anchor.y-q.y,n=Math.hypot(dx,dy);
                    if(n>EPS) for(double sign:new double[]{-1,1}) {
                        double offset=r+4;
                        Coordinate c=new Coordinate(q.x+sign*dx/n*offset,q.y+sign*dy/n*offset);
                        points.putIfAbsent(key(c),c);
                    }
                }
                // Local normals provide explicit candidates for straight atomic crossings.
                Geometry edge=obstacle.geometry.getDimension()==2?obstacle.geometry.getBoundary():obstacle.geometry;
                for(int part=0;part<edge.getNumGeometries();part++) {
                    Coordinate[] c=edge.getGeometryN(part).getCoordinates();
                    for(int i=1;i<c.length;i++) {
                        double len=c[i-1].distance(c[i]);if(len<EPS)continue;
                        int samples=Math.min(32,Math.max(1,(int)Math.ceil(len/(40.0/(1<<level)))));
                        for(int sample=0;sample<samples;sample++) {
                            double f=(sample+.5)/samples,x=c[i-1].x+(c[i].x-c[i-1].x)*f,y=c[i-1].y+(c[i].y-c[i-1].y)*f;
                            double nx=-(c[i].y-c[i-1].y)/len,ny=(c[i].x-c[i-1].x)/len;
                            for(double s:new double[]{-1,1}) {
                                Coordinate p=new Coordinate(x+s*nx*(r+4),y+s*ny*(r+4));
                                if(area.contains(p))points.putIfAbsent(key(p),p);
                            }
                        }
                    }
                }
            }
        }
        if(target!=null)for(PlanningContext.Feature own:context.ownBuildings(target)) {
            Coordinate t=target.geometry.getCoordinate();
            // All equal nearest boundary projections are candidates (including symmetric buildings).
            Geometry boundary=own.geometry.getBoundary();double nearest=boundary.distance(target.geometry);
            for(int part=0;part<boundary.getNumGeometries();part++) {
                Coordinate[] c=boundary.getGeometryN(part).getCoordinates();
                for(int i=1;i<c.length;i++) {
                    Coordinate q=new LineSegment(c[i-1],c[i]).closestPoint(t);
                    double n=q.distance(t);if(Math.abs(n-nearest)>.0001)continue;
                    double dx=q.x-t.x,dy=q.y-t.y;
                    if(n<EPS) {dx=-(c[i].y-c[i-1].y);dy=c[i].x-c[i-1].x;n=Math.hypot(dx,dy);}
                    double offset=rules.radius(own,d)+.05;
                    for(double sign:new double[]{1,-1}) {
                        Coordinate p=new Coordinate(q.x+sign*dx/n*offset,q.y+sign*dy/n*offset);
                        if(own.geometry.distance(point(p))>=rules.radius(own,d)-EPS){points.putIfAbsent(key(p),p);terminalLinks.add(key(p));}
                    }
                }
            }
        }
        // Stable order makes GeoJSON feature/vertex ordering irrelevant.
        List<Coordinate> result=new ArrayList<>(List.of(start,goal));
        points.remove(key(start));points.remove(key(goal));
        List<Coordinate> rest=new ArrayList<>(points.values());rest.sort(Comparator.comparingDouble((Coordinate c)->c.x).thenComparingDouble(c->c.y));
        result.addAll(rest);return result;
    }
    private LineString search(List<Coordinate> v,int d,double maxLength,PlanningContext.Feature target,PlanningContext.Root root,Profile profile,Geometry avoid,int level) {
        PriorityQueue<Label> open=new PriorityQueue<>(Comparator.comparingDouble((Label l)->l.f).thenComparingDouble(l->l.cost).thenComparingLong(l->l.sequence));
        Map<Long,List<Label>> labels=new HashMap<>();
        Map<Integer,List<Integer>> neighbors=new HashMap<>();
        Map<Long,GeometryRules.Check> cache=new HashMap<>();
        STRtree vertexIndex=new STRtree();
        Map<String,Integer> vertexIds=new HashMap<>();for(int i=0;i<v.size();i++)vertexIds.put(key(v.get(i)),i);
        Envelope graphEnvelope=new Envelope();
        for(int i=0;i<v.size();i++){vertexIndex.insert(new Envelope(v.get(i)),i);graphEnvelope.expandToInclude(v.get(i));}vertexIndex.build();
        long sequence=0;int expansions=0;
        open.add(new Label(0,-1,0,0,heuristic(v.get(0).distance(v.get(1)),d,profile),null,sequence++));
        while(!open.isEmpty()&&!budget.exhausted()&&expansions<budget.options.maxExpansionsPerRoute) {
            Label current=open.poll();if(current.stale)continue;
            expansions++;budget.expansions++;
            if(current.vertex==1) {
                List<Coordinate> path=new ArrayList<>();for(Label l=current;l!=null;l=l.parent)path.add(v.get(l.vertex));Collections.reverse(path);
                LineString result=line(path.toArray(new Coordinate[0]));
                if(rules.check(result,d,target,root).valid)return result;
                continue;
            }
            List<Integer> next=neighbors.computeIfAbsent(current.vertex,idx->{
                int needed=Math.min(v.size()-1,budget.options.neighborCount*(1<<level));
                double radius=10,maximum=Math.hypot(graphEnvelope.getWidth(),graphEnvelope.getHeight())+1;
                List<Integer> order;
                while(true){
                    Envelope query=new Envelope(v.get(idx));query.expandBy(radius);
                    order=new ArrayList<>();
                    for(Object item:vertexIndex.query(query)){int i=(Integer)item;if(i!=0&&i!=idx&&v.get(idx).distance(v.get(i))<=radius)order.add(i);}
                    if(order.size()>=needed||radius>=maximum)break;radius=Math.min(maximum,radius*2);
                }
                order.sort(Comparator.<Integer>comparingDouble(i->v.get(idx).distance(v.get(i))).thenComparingInt(i->i));
                int size=Math.min(order.size(),budget.options.neighborCount*(1<<level));
                List<Integer> picked=new ArrayList<>(order.subList(0,size));if(idx!=1&&!picked.contains(1))picked.add(1);
                Set<String> forced=new TreeSet<>(boundaryLinks.getOrDefault(key(v.get(idx)),Set.of()));forced.addAll(terminalLinks);
                for(String k:forced){Integer i=vertexIds.get(k);if(i!=null&&i!=idx&&i!=0&&!picked.contains(i))picked.add(i);}return picked;
            });
            for(int to:next) {
                if(budget.exhausted())return null;
                if(to==current.previous)continue;
                Coordinate a=v.get(current.vertex),b=v.get(to);
                if(current.previous>=0&&turn(v.get(current.previous),a,b)>90+1e-7)continue;
                double len=a.distance(b),newLength=current.length+len;
                if(newLength+b.distance(v.get(1))>maxLength+EPS)continue;
                long pair=((long)Math.min(current.vertex,to)<<32)|Math.max(current.vertex,to);
                GeometryRules.Check check=cache.get(pair);
                if(check==null) {
                    budget.transitions++;
                    check=rules.check(line(a,b),d,to==1?target:null,current.vertex==0?root:null);
                    if(cache.size()<100000)cache.put(pair,check);
                }
                if(!check.valid)continue;
                double cost=current.cost+weight(check.weightedLength*PipeCatalog.PRICE[d],len,profile);
                if(avoid!=null) {
                    double overlap=line(a,b).intersection(avoid).getLength();
                    cost+=heuristic(overlap,d,profile)*.35;
                }
                long state=((long)current.vertex<<32)|to;
                List<Label> frontier=labels.computeIfAbsent(state,k->new ArrayList<>());
                final double nc=cost,nl=newLength;
                if(frontier.stream().anyMatch(l->!l.stale&&l.cost<=nc+1e-10&&l.length<=nl+EPS))continue;
                for(Label l:frontier)if(nc<=l.cost+1e-10&&nl<=l.length+EPS)l.stale=true;
                frontier.removeIf(l->l.stale);
                Label label=new Label(to,current.vertex,newLength,cost,cost+heuristic(b.distance(v.get(1)),d,profile),current,sequence++);
                frontier.add(label);open.add(label);
            }
        }return null;
    }
    static double weight(double cost,double length,Profile profile) {return profile==Profile.ECONOMIC?cost:profile==Profile.COMPACT?length:PipeCatalog.score(cost,length);}
    private double heuristic(double distance,int d,Profile profile){return weight(distance*PipeCatalog.PRICE[d],distance,profile);}
}
