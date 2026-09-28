package ru.heatplanner;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import ru.heatplanner.solver.NetworkPlanner;
import ru.heatplanner.solver.SearchOptions;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicBoolean;

/** Minimal adapter to the existing Spring application. No persisted jobs or new service. */
@RestController
public class PlanningController {
    private final ObjectMapper mapper;
    private final Semaphore capacity=new Semaphore(1);
    private final ConcurrentHashMap<String,AtomicBoolean> cancellations=new ConcurrentHashMap<>();
    public PlanningController(ObjectMapper mapper){this.mapper=mapper;}
    @PostMapping("/api/plan")
    public ResponseEntity<?> plan(@RequestBody JsonNode request) {
        String id=request.path("requestId").asText();
        if(!id.matches("[a-zA-Z0-9-]{8,80}"))return ResponseEntity.badRequest().body(Map.of("error","Не задан requestId расчёта"));
        if(!capacity.tryAcquire())return ResponseEntity.status(429).body(Map.of("error","Другой расчёт ещё выполняется"));
        AtomicBoolean cancelled=new AtomicBoolean();cancellations.put(id,cancelled);
        try {
            SearchOptions options=request.has("options")?mapper.treeToValue(request.get("options"),SearchOptions.class):new SearchOptions();
            return ResponseEntity.ok(new NetworkPlanner(mapper).solveNetwork(request.path("dataset"),request.path("osm"),options,cancelled::get));
        } catch(IllegalArgumentException e) {return ResponseEntity.badRequest().body(Map.of("error",e.getMessage()));}
        catch(com.fasterxml.jackson.core.JsonProcessingException e){return ResponseEntity.badRequest().body(Map.of("error","Некорректные параметры поиска"));}
        finally {cancellations.remove(id);capacity.release();}
    }
    @DeleteMapping("/api/plan/{requestId}")
    public ResponseEntity<?> cancel(@PathVariable String requestId){AtomicBoolean flag=cancellations.get(requestId);if(flag!=null)flag.set(true);return ResponseEntity.ok(Map.of("cancelled",flag!=null));}
}
