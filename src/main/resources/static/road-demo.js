// TEMPORARY demonstration; remove this adapter without removing road-enrichment.js.
window.mountRoadDemo = function (map, store, legendItems, isInputTab) {
    const source = new ol.source.Vector({wrapX: false,
        attributions: '<a href="https://www.openstreetmap.org/copyright" target="_blank" rel="noopener">© OpenStreetMap contributors</a>'});
    const layer = new ol.layer.Vector({source, zIndex: 350, visible: false,
        style: new ol.style.Style({stroke: new ol.style.Stroke({color: "#ffffff", width: 2.5})})});
    layer.set("temporaryRoadDemo", true);
    map.addLayer(layer);
    const row = document.createElement("label");
    row.className = "legend-row"; row.dataset.layer = "osm-roads-demo";
    const check = document.createElement("input");
    check.type = "checkbox"; check.checked = true; check.disabled = true;
    check.setAttribute("aria-label", "Дороги OSM (демо)");
    const swatch = document.createElement("span"); swatch.className = "swatch"; swatch.style.background = "#ffffff";
    const title = document.createElement("span"); title.textContent = "Дороги OSM (демо)";
    row.append(check, swatch, title); legendItems.append(row);
    const info = document.createElement("div"); info.className = "road-import-info";
    const message = document.createElement("p"); message.id = "roadImportStatus"; message.className = "muted";
    message.setAttribute("role", "status"); message.setAttribute("aria-live", "polite");
    const retry = document.createElement("button"); retry.type = "button"; retry.textContent = "Повторить загрузку дорог"; retry.hidden = true;
    retry.addEventListener("click", () => store.load());
    info.append(message, retry); document.getElementById("inputSummary").append(info);
    let loadedAt = null;
    function sync() {
        const state = store.snapshot();
        row.hidden = !isInputTab();
        check.disabled = state.status !== "ready";
        layer.setVisible(isInputTab() && check.checked && state.status === "ready");
    }
    check.addEventListener("change", sync);
    function update() {
        const state = store.snapshot();
        retry.hidden = state.status !== "error";
        if (state.status === "ready") {
            if (loadedAt !== state.fetchedAt) {
                source.clear(); source.addFeatures(new ol.format.GeoJSON().readFeatures(store.getRoads(), {
                    dataProjection: "EPSG:4326", featureProjection: "EPSG:3857"}));
                loadedAt = state.fetchedAt;
            }
            message.textContent = state.count ? `Дороги OSM: ${state.count} объектов в памяти. Белые линии — оси дорог, не границы проезжей части.` : "В этой области дополнительные автомобильные дороги OSM не найдены.";
        } else if (state.status === "error") {
            message.textContent = `Дороги не загружены: ${state.error} Исходные данные доступны.`;
        } else message.textContent = "Загружаем дороги OSM в пределах исходных данных…";
        sync();
    }
    store.subscribe(update); update();
    // Loading does not depend on the active tab and never changes the camera extent.
    store.load();
    return sync;
};
