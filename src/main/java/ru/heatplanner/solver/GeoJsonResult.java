package ru.heatplanner.solver;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.*;
import org.locationtech.jts.geom.*;
import org.locationtech.jts.linearref.LengthIndexedLine;
import java.util.*;

final class GeoJsonResult {
    private final PlanningContext context;private final ObjectMapper mapper;
    private final Set<String> used=new HashSet<>();private int serial;
    GeoJsonResult(PlanningContext context,ObjectMapper mapper){this.context=context;this.mapper=mapper;used.addAll(context.ids);}
    private TextNode id(String variant){String value;do{value="new:"+variant+":"+(++serial);}while(!used.add("s:"+value));return TextNode.valueOf(value);}
    JsonNode write(NetworkCandidate n,int rank) {
        String variant=String.valueOf(rank);
        ObjectNode result=mapper.createObjectNode();result.put("type","FeatureCollection");result.put("validation_status","VALID");
        result.put("description","Проверенная схема: врезок "+n.nodes.values().stream().filter(x->x.root!=null).count()+", общих разветвлений "+n.nodes.values().stream().filter(x->n.children(x.key).size()>1).count());
        result.set("input_osm_metadata",context.osm==null?NullNode.instance:context.osm.deepCopy());
        ArrayNode features=result.putArray("features");Map<String,JsonNode> nodeIds=new HashMap<>();
        for(NetworkCandidate.Node node:n.nodes.values()) {
            if(node.target!=null){nodeIds.put(node.key,node.target.id);continue;}
            if(node.root!=null&&node.root.chamber!=null){nodeIds.put(node.key,node.root.chamber.id);continue;}
            JsonNode nodeId=id(variant);nodeIds.put(node.key,nodeId);
            boolean chamber=node.root!=null||n.children(node.key).size()>1;
            ObjectNode p=properties(nodeId,chamber?"heat_chamber":"technical_node",variant);
            if(chamber) {
                int d=node.root==null?0:context.existingDiameter(node.point);
                for(NetworkCandidate.Edge e:n.edges)if(e.from.equals(node.key)||e.to.equals(node.key))d=Math.max(d,e.diameter);
                p.put("diameter",PipeCatalog.DN[d]);p.put("cost",PipeCatalog.chamber(d));
                if(node.root!=null)p.set("existing_object_id",node.root.pipe.id);
            }
            features.add(feature(MetricGeometry.point(node.point),p));
        }
        GeometryRules rules=new GeometryRules(context);
        for(NetworkCandidate.Edge e:n.edges) {
            GeometryRules.Check check=rules.check(e.route,e.diameter,n.nodes.get(e.to).target,n.nodes.get(e.from).root);
            if(!check.valid)throw new IllegalStateException("Непроверенная трасса при экспорте");
            List<GeometryRules.Interval> parts=GeometryRules.costIntervals(e.route.getLength(),check.intervals);
            LengthIndexedLine ref=new LengthIndexedLine(e.route);JsonNode from=nodeIds.get(e.from);
            for(int i=0;i<parts.size();i++) {
                GeometryRules.Interval part=parts.get(i);JsonNode to=i==parts.size()-1?nodeIds.get(e.to):id(variant);
                if(i<parts.size()-1)features.add(feature(MetricGeometry.point(ref.extractPoint(part.to)),properties(to,"technical_node",variant)));
                Geometry geometry=ref.extractLine(part.from,part.to);
                ObjectNode p=properties(id(variant),"heat_network",variant);p.set("start_node_id",from);p.set("end_node_id",to);
                p.put("flow_tph",e.flow);p.put("diameter",PipeCatalog.DN[e.diameter]);p.put("length",geometry.getLength());
                p.put("laying_method",part.k>1?"special":"base");p.put("special_coefficient",part.k);
                p.putNull("depth_start");p.putNull("depth_end");p.put("cost",geometry.getLength()*PipeCatalog.PRICE[e.diameter]*part.k);
                ArrayNode obstacleIds=p.putArray("crossed_obstacle_ids");
                for(GeometryRules.Interval event:check.intervals)if((part.from+part.to)/2>=event.from&&(part.from+part.to)/2<=event.to)
                    context.obstacles.stream().filter(o->o.key.equals(event.obstacle)).findFirst().ifPresent(o->obstacleIds.add(o.id));
                features.add(feature(geometry,p));from=to;
            }
        }
        ObjectNode summary=properties(id(variant),"variant_summary",variant);
        summary.put("rank",rank);summary.put("construction_cost",n.cost);summary.put("chamber_construction_cost",n.chamberCost);
        summary.put("existing_chamber_tie_in_count",n.tieIns);summary.put("existing_chamber_tie_in_cost",n.tieInCost);
        summary.put("unconnected_penalty",n.penalty);summary.put("calculated_cost",n.cost+n.penalty);summary.put("new_network_length",n.length);summary.put("score",n.score);
        ArrayNode pending=summary.putArray("unconnected_oks_ids");for(PlanningContext.Feature t:context.targets)if(n.pending.contains(t.key))pending.add(t.id);
        features.add(feature(null,summary));return result;
    }
    private ObjectNode properties(JsonNode id,String type,String variant) {ObjectNode p=mapper.createObjectNode();p.set("id",id);p.put("object_type",type);p.put("variant_id",variant);return p;}
    private ObjectNode feature(Geometry geometry,ObjectNode p) {ObjectNode f=mapper.createObjectNode();f.put("type","Feature");f.set("geometry",geometry==null?NullNode.instance:context.metric.write(geometry,mapper));f.set("properties",p);return f;}
}
