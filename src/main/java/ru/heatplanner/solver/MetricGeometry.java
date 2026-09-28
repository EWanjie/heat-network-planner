package ru.heatplanner.solver;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.locationtech.jts.geom.*;
import org.locationtech.jts.io.geojson.GeoJsonReader;
import org.locationtech.jts.io.geojson.GeoJsonWriter;
import org.locationtech.proj4j.*;

public final class MetricGeometry {
    public static final GeometryFactory GF = new GeometryFactory();
    public static final double EPS = 1e-4; // 0.1 mm: includes the GeoJSON round-trip precision.
    private final CoordinateTransform forward, inverse;
    public MetricGeometry() {
        CRSFactory f=new CRSFactory();
        CoordinateReferenceSystem wgs=f.createFromParameters("EPSG:4326", "+proj=longlat +datum=WGS84 +no_defs");
        CoordinateReferenceSystem utm=f.createFromParameters("EPSG:32637", "+proj=utm +zone=37 +datum=WGS84 +units=m +no_defs");
        CoordinateTransformFactory t=new CoordinateTransformFactory();
        forward=t.createTransform(wgs,utm); inverse=t.createTransform(utm,wgs);
    }
    public Geometry project(Geometry geometry, boolean toMeters) {
        Geometry copy=geometry.copy();
        copy.apply((CoordinateFilter)c -> {
            ProjCoordinate result=new ProjCoordinate();
            (toMeters ? forward : inverse).transform(new ProjCoordinate(c.x,c.y),result);
            c.x=result.x; c.y=result.y; c.setZ(Double.NaN);
        });
        copy.geometryChanged(); return copy;
    }
    public Geometry read(JsonNode node) {
        try {
            Geometry geo=new GeoJsonReader(GF).read(node.toString());
            for(Coordinate c:geo.getCoordinates()) if(!Double.isFinite(c.x)||!Double.isFinite(c.y)||c.x< -180||c.x>180||c.y<0||c.y>84)
                throw new IllegalArgumentException("Координаты вне области северной UTM");
            if(geo.isEmpty() || !geo.isValid()) throw new IllegalArgumentException("Пустая или некорректная геометрия");
            Geometry metric=project(geo,true);
            if(!metric.isValid()) throw new IllegalArgumentException("Некорректная метрическая геометрия");
            return metric;
        } catch(Exception e) { throw new IllegalArgumentException("Некорректная GeoJSON-геометрия: "+e.getMessage(),e); }
    }
    public JsonNode write(Geometry metric, ObjectMapper mapper) {
        try {
            GeoJsonWriter writer=new GeoJsonWriter(10); writer.setEncodeCRS(false);
            return mapper.readTree(writer.write(project(metric,false)));
        } catch(Exception e) {throw new IllegalStateException(e);}
    }
    public static LineString line(Coordinate... c) { return GF.createLineString(c); }
    public static Point point(Coordinate c) { return GF.createPoint(c); }
    public static String key(Coordinate c) {return Math.round(c.x*1000)+":"+Math.round(c.y*1000);}
    public static double turn(Coordinate a,Coordinate b,Coordinate c) {
        double ux=b.x-a.x,uy=b.y-a.y,vx=c.x-b.x,vy=c.y-b.y;
        return Math.toDegrees(Math.acos(Math.max(-1,Math.min(1,(ux*vx+uy*vy)/(Math.hypot(ux,uy)*Math.hypot(vx,vy))))));
    }
}
