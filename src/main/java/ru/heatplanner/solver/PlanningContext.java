package ru.heatplanner.solver;

import com.fasterxml.jackson.databind.JsonNode;
import org.locationtech.jts.geom.*;
import org.locationtech.jts.index.strtree.STRtree;
import org.locationtech.jts.linearref.LengthIndexedLine;
import java.util.*;

final class PlanningContext {
    static final Set<String> HARD=Set.of("oks","park","social_area","prohibited_site","water","railway");
    static final Set<String> SPECIAL=Set.of("road","tram_tracks","gas_pipeline","power_cable","heat_network");
    static final class Feature {
        final JsonNode id, properties;
        final String key,type;
        final Geometry geometry;
        Feature(JsonNode id,JsonNode p,String type,Geometry geometry) {
            this.id=id;properties=p;this.type=type;this.geometry=geometry;
            key=id.isTextual()?"s:"+id.textValue():"n:"+id.decimalValue().stripTrailingZeros().toPlainString();
        }
        double flow() {return properties.path("flow_tph").asDouble();}
    }
    static final class Root {
        final Coordinate point;
        final Feature pipe, chamber;
        Root(Coordinate point,Feature pipe,Feature chamber) {this.point=point;this.pipe=pipe;this.chamber=chamber;}
        String key(){return chamber==null ? "pipe:"+pipe.key+":"+MetricGeometry.key(point) : "chamber:"+chamber.key;}
    }
    final MetricGeometry metric=new MetricGeometry();
    final List<Feature> obstacles=new ArrayList<>(), targets=new ArrayList<>(), pipes=new ArrayList<>(), chambers=new ArrayList<>();
    final List<Map<String,Object>> diagnostics=new ArrayList<>();
    final Set<String> diagnosticKeys=new HashSet<>();
    final Set<String> ids=new HashSet<>();
    final STRtree index=new STRtree();
    final JsonNode osm;
    final Geometry coverage;
    boolean invalidInput;
    final Envelope requestedCoverage=new Envelope();
    PlanningContext(JsonNode collection,JsonNode osm) {
        this.osm=osm;
        if(!collection.path("type").asText().equals("FeatureCollection") || !collection.path("features").isArray())
            throw new IllegalArgumentException("Ожидается GeoJSON FeatureCollection");
        for(JsonNode f:collection.path("features")) {
            JsonNode p=f.path("properties"),id=p.path("id");
            if(!id.isTextual()&&!id.isNumber()) throw new IllegalArgumentException("У каждого объекта должен быть строковый или числовой id");
            String type=p.path("object_type").asText();
            if(type.equals("restriction")) type=p.path("restriction_type").asText();
            if(type.equals("oks_existing")) type="oks";
            Feature feature=new Feature(id,p,type,metric.read(f.path("geometry")));
            if(!ids.add(feature.key)) throw new IllegalArgumentException("Повторный id: "+id);
            switch(type) {
                case "oks_connection_point":
                    if(!(feature.geometry instanceof Point) || !p.path("flow_tph").isNumber() || !Double.isFinite(feature.flow()) || feature.flow()<=0)
                        throw new IllegalArgumentException("Неверная точка подключения или расход: "+id);
                    targets.add(feature); break;
                case "heat_chamber":
                    if(!(feature.geometry instanceof Point)) throw new IllegalArgumentException("Камера должна быть точкой");
                    chambers.add(feature);break;
                case "heat_network":
                    if(!(feature.geometry instanceof LineString)) throw new IllegalArgumentException("Существующая сеть должна быть LineString");
                    PipeCatalog.index(p.path("diameter").asInt()); pipes.add(feature); obstacles.add(feature);break;
                case "source":break;
                default:
                    if(!HARD.contains(type)&&!SPECIAL.contains(type)) {
                        diagnostic("UNSUPPORTED_RESTRICTION",feature,"Тип ограничения не поддерживается"); invalidInput=true;
                    }
                    if(type.equals("oks")&&feature.geometry.getDimension()!=2) throw new IllegalArgumentException("ОКС должен быть полигоном");
                    if((type.equals("road")||type.equals("tram_tracks")) && feature.geometry.getDimension()==1) {
                        double width=p.path("width_m").asDouble(0);
                        if(Double.isFinite(width)&&width>0 && !p.path("width_source").asText().equals("unknown"))
                            feature=new Feature(id,p,type,feature.geometry.buffer(width/2,16));
                        else diagnostic("ROAD_WIDTH_REQUIRED",feature,"Ширина неизвестна; пересечение не может быть окончательно рассчитано");
                    }
                    obstacles.add(feature);
            }
        }
        Comparator<Feature> order=Comparator.comparing(f->f.key);
        targets.sort(order);pipes.sort(order);chambers.sort(order);obstacles.sort(order);
        for(Feature f:obstacles) index.insert(f.geometry.getEnvelopeInternal(),f);
        index.build();
        Geometry cover=null;
        if(osm!=null && osm.path("required").asBoolean(false)) {
            if(!osm.path("status").asText().equals("ready")) {diagnostic("OSM_DATA_UNAVAILABLE",null,"Дополнительные ограничения OSM не загружены");invalidInput=true;}
            JsonNode b=osm.path("bbox");
            if(b.isArray()&&b.size()==4) {
                Coordinate[] ring={new Coordinate(b.get(0).asDouble(),b.get(1).asDouble()),new Coordinate(b.get(2).asDouble(),b.get(1).asDouble()),new Coordinate(b.get(2).asDouble(),b.get(3).asDouble()),new Coordinate(b.get(0).asDouble(),b.get(3).asDouble()),new Coordinate(b.get(0).asDouble(),b.get(1).asDouble())};
                cover=metric.project(MetricGeometry.GF.createPolygon(ring),true);
            } else {diagnostic("OSM_COVERAGE_INCOMPLETE",null,"Не задано покрытие OSM");invalidInput=true;}
        }
        coverage=cover;
    }
    void diagnostic(String code, Feature feature,String message) {
        String key=code+":"+(feature==null?"":feature.key);
        if(diagnosticKeys.add(key)) {
            Map<String,Object> d=new LinkedHashMap<>(); d.put("code",code);d.put("message",message);
            if(feature!=null) d.put("objectId",feature.id);diagnostics.add(d);
        }
    }
    void needCoverage(Geometry route) {
        Geometry geographic=metric.project(route.buffer(15),false);
        requestedCoverage.expandToInclude(geographic.getEnvelopeInternal());
        diagnostic("OSM_COVERAGE_INCOMPLETE",null,"Коридор выходит за загруженную область; требуется догрузка ограничений");
    }
    @SuppressWarnings("unchecked") List<Feature> nearby(Geometry g,double radius) {
        Envelope e=new Envelope(g.getEnvelopeInternal());e.expandBy(radius);return index.query(e);
    }
    List<Feature> ownBuildings(Feature target) {
        List<Feature> result=new ArrayList<>();
        for(Feature f:nearby(target.geometry,0)) if(f.type.equals("oks")&&f.geometry.covers(target.geometry)) result.add(f);
        result.sort(Comparator.comparing(f->f.key));return result;
    }
    int existingDegree(Coordinate p) {
        int count=0;
        for(Feature f:pipes) {
            Coordinate[] cs=f.geometry.getCoordinates();
            for(int i=1;i<cs.length;i++) {
                LineSegment s=new LineSegment(cs[i-1],cs[i]);
                if(s.distance(p)<=.01) count+=(p.distance(cs[i-1])<=.01||p.distance(cs[i])<=.01)?1:2;
            }
        }
        return count;
    }
    int existingDiameter(Coordinate p) {
        int d=0;for(Feature f:pipes) if(f.geometry.distance(MetricGeometry.point(p))<=.01) d=Math.max(d,PipeCatalog.index(f.properties.path("diameter").asInt()));return d;
    }
    List<Root> roots(Coordinate target, SearchOptions options, int refinement) {
        Map<String,Root> all=new TreeMap<>();
        for(Feature pipe:pipes) {
            LengthIndexedLine ref=new LengthIndexedLine(pipe.geometry);
            List<Coordinate> points=new ArrayList<>();
            points.add(ref.extractPoint(ref.project(target)));
            points.addAll(Arrays.asList(pipe.geometry.getCoordinates()));
            double step=options.tieInSamplingStep/Math.pow(2,refinement);
            for(double m=step;m<pipe.geometry.getLength();m+=step) points.add(ref.extractPoint(m));
            for(Coordinate p:points) {
                List<Feature> near=new ArrayList<>();
                for(Feature chamber:chambers) if(chamber.geometry.getCoordinate().distance(p)<=10+MetricGeometry.EPS && existingDegree(chamber.geometry.getCoordinate())>0 && existingDegree(chamber.geometry.getCoordinate())<4) near.add(chamber);
                if(near.isEmpty()) {
                    Root r=new Root(p,pipe,null);all.put(r.key(),r);
                } else for(Feature c:near) {Root r=new Root(c.geometry.getCoordinate(),pipe,c);all.put(r.key(),r);}
            }
        }
        for(Feature c:chambers) if(existingDegree(c.geometry.getCoordinate())>0&&existingDegree(c.geometry.getCoordinate())<4) {
            Feature pipe=pipes.stream().filter(p->p.geometry.distance(c.geometry)<=.01).findFirst().orElse(null);
            if(pipe!=null) {Root r=new Root(c.geometry.getCoordinate(),pipe,c);all.put(r.key(),r);}
        }
        List<Root> result=new ArrayList<>(all.values());
        result.sort(Comparator.<Root>comparingDouble(r->r.point.distance(target)).thenComparing(Root::key));
        int max=Math.min(options.maxTieInCandidateCount,options.initialTieInCandidateCount*(1<<refinement));
        return result.subList(0,Math.min(max,result.size()));
    }
}
