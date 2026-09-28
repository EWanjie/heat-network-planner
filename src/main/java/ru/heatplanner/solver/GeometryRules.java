package ru.heatplanner.solver;

import org.locationtech.jts.geom.*;
import org.locationtech.jts.linearref.LengthIndexedLine;
import org.locationtech.jts.operation.distance.DistanceOp;
import java.util.*;
import static ru.heatplanner.solver.MetricGeometry.*;

/** Exact checks on complete segments, with explicitly bounded local exceptions. */
final class GeometryRules {
    static final class Interval {
        final double from,to,k;
        final String obstacle;
        Interval(double from,double to,double k,String obstacle){this.from=from;this.to=to;this.k=k;this.obstacle=obstacle;}
    }
    static final class Check {
        boolean valid=true;
        String reason="";
        double weightedLength;
        final List<Interval> intervals=new ArrayList<>();
        Check fail(String reason){valid=false;this.reason=reason;return this;}
    }
    final PlanningContext context;
    GeometryRules(PlanningContext context){this.context=context;}
    double clearance(PlanningContext.Feature f,int d) {
        switch(f.type) {
            case "oks":return PipeCatalog.DN[d]<500?5:PipeCatalog.DN[d]<900?7:9;
            case "road":case "tram_tracks":return 1.5;
            case "gas_pipeline":return 2+.20;
            case "power_cable":return 2+.10;
            case "heat_network":return 1+PipeCatalog.WIDTH[PipeCatalog.index(f.properties.path("diameter").asInt())]/2;
            default:return 1;
        }
    }
    double radius(PlanningContext.Feature f,int d){return clearance(f,d)+PipeCatalog.WIDTH[d]/2;}
    Check check(LineString route,int d,PlanningContext.Feature target,PlanningContext.Root root) {
        Check result=new Check();
        if(route.getLength()<EPS) return result.fail("ZERO_LENGTH");
        Coordinate[] cs=route.getCoordinates();
        for(int i=1;i<cs.length-1;i++) if(turn(cs[i-1],cs[i],cs[i+1])>90+1e-4) return result.fail("TURN_OVER_90");
        LengthIndexedLine indexed=new LengthIndexedLine(route);
        for(PlanningContext.Feature f:context.nearby(route,12)) {
            double r=radius(f,d);
            if(f.geometry.distance(route)>=r-EPS) continue;
            if(PlanningContext.HARD.contains(f.type)) {
                if(f.type.equals("oks")&&target!=null&&f.geometry.covers(target.geometry)&&validTerminal(route,target,f,r)) continue;
                return result.fail("CLEARANCE:"+f.key);
            }
            if(!PlanningContext.SPECIAL.contains(f.type)) return result.fail("UNSUPPORTED_RESTRICTION");
            Geometry cross=route.intersection(f.geometry);
            List<Interval> exceptions=new ArrayList<>();
            if(root!=null&&f.type.equals("heat_network")&&f.geometry.distance(point(root.point))<.01&&validTieIn(route,root,f)) {
                // Only the final straight approach within the pipe's clearance buffer is exempt.
                Coordinate[] routePoints=route.getCoordinates();
                boolean rootAtStart=routePoints[0].distance(root.point)<.01;
                LineString approach=rootAtStart?line(routePoints[0],routePoints[1]):line(routePoints[routePoints.length-1],routePoints[routePoints.length-2]);
                Geometry inside=approach.intersection(f.geometry.buffer(r/Math.cos(Math.PI/64)+EPS,16));
                for(int i=0;i<inside.getNumGeometries();i++) {
                    Geometry part=inside.getGeometryN(i);
                    if(part.distance(point(root.point))<.01) {
                        LengthIndexedLine approachIndex=new LengthIndexedLine(approach);
                        double upper=indices(part,approachIndex)[1],lower=0;
                        for(int step=0;step<45;step++) {
                            double middle=(lower+upper)/2;
                            if(f.geometry.distance(point(approachIndex.extractPoint(middle)))<r)lower=middle;else upper=middle;
                        }
                        double exit=upper;
                        exceptions.add(rootAtStart?new Interval(0,exit,1,f.key):new Interval(route.getLength()-exit,route.getLength(),1,f.key));
                    }
                }
                cross=cross.difference(point(root.point).buffer(.001));
            }
            if(!cross.isEmpty()) {
                if((f.type.equals("road")||f.type.equals("tram_tracks"))&&f.geometry.getDimension()==1) {
                    context.diagnostic("ROAD_WIDTH_REQUIRED",f,"Найдено пересечение линии без ширины; необходим полигон или пригодная ширина");
                    return result.fail("ROAD_WIDTH_REQUIRED");
                }
                for(int j=0;j<cross.getNumGeometries();j++) {
                    Geometry piece=cross.getGeometryN(j);
                    if(f.geometry.getDimension()==1&&piece.getDimension()>0) return result.fail("ALONG_EXISTING_LINE");
                    double[] bounds=indices(piece,indexed);
                    double pad=(f.type.equals("road")||f.type.equals("tram_tracks"))?3:2;
                    double from=bounds[0]-pad,to=bounds[1]+pad;
                    if(from< -EPS||to>route.getLength()+EPS) return result.fail("INCOMPLETE_SPECIAL");
                    Geometry straight=indexed.extractLine(Math.max(0,from),Math.min(route.getLength(),to));
                    if(!straight(straight)) return result.fail("BEND_INSIDE_SPECIAL");
                    if(f.type.equals("road")||f.type.equals("tram_tracks")) {
                        Coordinate entry=indexed.extractPoint(bounds[0]);
                        if(angleToBoundary(straight,f.geometry.getBoundary(),entry)<45-1e-4) return result.fail("CROSSING_ANGLE");
                        // Tangency is not a crossing through the area.
                        if(piece.getLength()<EPS) return result.fail("TANGENT_SPECIAL");
                    }
                    Interval event=new Interval(Math.max(0,from),Math.min(route.getLength(),to),coefficient(f.type),f.key);
                    exceptions.add(event);result.intervals.add(event);
                }
            }
            // Check all remaining parts, not merely endpoints. No silent enlargement of a special interval.
            List<Double> cuts=new ArrayList<>(List.of(0.0,route.getLength()));
            for(Interval e:exceptions){cuts.add(e.from);cuts.add(e.to);}Collections.sort(cuts);
            for(int j=1;j<cuts.size();j++) {
                double a=cuts.get(j-1),b=cuts.get(j);if(b-a<EPS) continue;
                double mid=(a+b)/2;
                if(exceptions.stream().anyMatch(e->mid>=e.from-EPS&&mid<=e.to+EPS)) continue;
                Geometry part=indexed.extractLine(a,b);
                if(part.distance(f.geometry)<r-EPS) {
                    if(!exceptions.isEmpty()) context.diagnostic("RULE_CONFLICT",f,"Специальный интервал не покрывает необходимую зону отступа для выбранного ДУ");
                    return result.fail(exceptions.isEmpty()?"CLEARANCE":"RULE_CONFLICT");
                }
            }
        }
        for(Interval e:costIntervals(route.getLength(),result.intervals)) result.weightedLength+=(e.to-e.from)*e.k;
        return result;
    }
    static double coefficient(String type) {
        switch(type){case "road":return 1.60;case "tram_tracks":return 1.75;case "gas_pipeline":return 1.25;case "power_cable":return 1.15;default:return 1.05;}
    }
    static List<Interval> costIntervals(double length,List<Interval> events) {
        TreeSet<Double> cuts=new TreeSet<>(List.of(0.0,length));
        for(Interval e:events){cuts.add(e.from);cuts.add(e.to);}
        List<Double> c=new ArrayList<>(cuts);List<Interval> result=new ArrayList<>();
        for(int i=1;i<c.size();i++) {
            double a=c.get(i-1),b=c.get(i),mid=(a+b)/2,k=1;
            for(Interval e:events) if(mid>=e.from&&mid<=e.to) k=Math.max(k,e.k);
            if(b-a>EPS) result.add(new Interval(a,b,k,""));
        }return result;
    }
    private boolean validTerminal(LineString route,PlanningContext.Feature target,PlanningContext.Feature own,double radius) {
        Coordinate t=target.geometry.getCoordinate();Coordinate[] cs=route.getCoordinates();
        boolean atEnd=cs[cs.length-1].distance(t)<EPS,atStart=cs[0].distance(t)<EPS;
        if(!atEnd&&!atStart) return false;
        int last=atEnd?cs.length-1:0, prev=atEnd?last-1:1;
        LineString terminal=line(cs[prev],cs[last]);
        double nearest=own.geometry.getBoundary().distance(target.geometry);
        Geometry entry=terminal.intersection(own.geometry.getBoundary());
        if(entry.isEmpty()||Arrays.stream(entry.getCoordinates()).noneMatch(p->Math.abs(p.distance(t)-nearest)<EPS)) return false;
        Geometry hit=terminal.intersection(own.geometry);
        if(hit.getNumGeometries()!=1 || hit.distance(target.geometry)>EPS) return false;
        if(own.geometry.distance(point(cs[prev]))<radius-EPS) return false;
        if(cs.length>2) {
            Coordinate[] rest=atEnd?Arrays.copyOf(cs,cs.length-1):Arrays.copyOfRange(cs,1,cs.length);
            if(line(rest).distance(own.geometry)<radius-EPS) return false;
        }return true;
    }
    private boolean validTieIn(LineString route,PlanningContext.Root root,PlanningContext.Feature pipe) {
        Coordinate[] c=route.getCoordinates();boolean start=c[0].distance(root.point)<.01,end=c[c.length-1].distance(root.point)<.01;
        if(!start&&!end) return false;
        LineString approach=start?line(c[0],c[1]):line(c[c.length-1],c[c.length-2]);
        Geometry cross=approach.intersection(pipe.geometry);
        if(!cross.isEmpty()&&cross.getDimension()!=0) return false;
        for(Coordinate p:cross.getCoordinates()) if(p.distance(root.point)>.01) return false;
        Coordinate other=start?c[1]:c[c.length-2];
        if(pipe.geometry.distance(point(other))<radius(pipe,0)) return false;
        return true;
    }
    static double[] indices(Geometry g,LengthIndexedLine line) {
        double min=Double.POSITIVE_INFINITY,max=Double.NEGATIVE_INFINITY;
        for(Coordinate c:g.getCoordinates()){double v=line.project(c);min=Math.min(min,v);max=Math.max(max,v);}return new double[]{min,max};
    }
    static boolean straight(Geometry g) {
        Coordinate[] c=g.getCoordinates();if(c.length<2)return false;
        return Math.abs(c[0].distance(c[c.length-1])-g.getLength())<1e-6;
    }
    private double angleToBoundary(Geometry route,Geometry boundary,Coordinate at) {
        Coordinate[] r=route.getCoordinates();double best=90;boolean found=false;
        for(int g=0;g<boundary.getNumGeometries();g++) {
            Coordinate[] c=boundary.getGeometryN(g).getCoordinates();
            for(int i=1;i<c.length;i++) if(new LineSegment(c[i-1],c[i]).distance(at)<.001) {
                double ux=r[r.length-1].x-r[0].x,uy=r[r.length-1].y-r[0].y,vx=c[i].x-c[i-1].x,vy=c[i].y-c[i-1].y;
                double cos=Math.abs((ux*vx+uy*vy)/(Math.hypot(ux,uy)*Math.hypot(vx,vy)));
                best=Math.min(best,Math.toDegrees(Math.acos(Math.min(1,cos))));found=true;
            }
        }return found?best:0;
    }
}
