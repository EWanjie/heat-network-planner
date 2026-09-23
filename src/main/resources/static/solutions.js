// Решения подключения на странице результатов: запрос расчёта (POST /api/plan), отрисовка выбранного решения
// на карте OpenLayers и подробности в боковой панели. Подключается перед results.js и работает через window.solutions:
//   attach(map, geojson)  — вызывается, когда карта построена;
//   start(file, ...)      — запускает расчёт для загруженного файла;
//   show(variant, box)    — рисует решение на карте и заполняет панель; hide() — убирает решение с карты;
//   download()            — скачивает GeoJSON со всеми решениями (кнопка «Выгрузить данные»).
window.solutions = (() => {
    const proj = {dataProjection: "EPSG:4326", featureProjection: "EPSG:3857"};
    const nf0 = new Intl.NumberFormat("ru-RU", {maximumFractionDigits: 0});
    const nf1 = new Intl.NumberFormat("ru-RU", {maximumFractionDigits: 1});
    const nf2 = new Intl.NumberFormat("ru-RU", {maximumFractionDigits: 2});
    const mln = rub => nf1.format(rub / 1e6) + " млн ₽";
    const meters = m => nf0.format(m) + " м";
    const esc = value => String(value ?? "").replace(/[&<>"']/g,
        ch => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"}[ch]));

    let map = null;
    let inputGeojson = null;
    let plan = null;
    let layers = [];
    let legend = null;

    // ---------- Запрос расчёта ----------
    async function start(file) {
        const body = new FormData();
        body.append("file", file);
        const response = await fetch("/api/plan", {method: "POST", body});
        if (!response.ok) {
            const text = await response.text();
            throw new Error(response.status >= 500 ? "Сервер не смог выполнить расчёт." : text);
        }
        plan = await response.json();
        return plan.meta.variants;
    }

    function attach(olMap, geojson) {
        map = olMap;
        inputGeojson = geojson;
    }

    function download() {
        if (!plan) return;
        const blob = new Blob([JSON.stringify(plan.geojson)], {type: "application/geo+json"});
        const link = document.createElement("a");
        link.href = URL.createObjectURL(blob);
        link.download = "heat-network-solutions.geojson";
        document.body.append(link);
        link.click();
        link.remove();
        setTimeout(() => URL.revokeObjectURL(link.href), 1000);
    }

    // ---------- Карта ----------
    const PIPE_COLORS = [[100, "#5eead4"], [150, "#38bdf8"], [250, "#818cf8"], [1e9, "#c084fc"]];
    const pipeColor = dn => (PIPE_COLORS.find(([limit]) => dn <= limit) || PIPE_COLORS[3])[1];
    const utilizationColor = u => u < 0.6 ? "#4ade80" : u < 0.85 ? "#fbbf24" : "#ff4d6d";
    const cache = new Map();

    function cached(key, make) {
        if (!cache.has(key)) cache.set(key, make());
        return cache.get(key);
    }

    function circle(radius, fill, stroke, width) {
        return cached(`c:${radius}:${fill}:${stroke}:${width}`, () => new ol.style.Style({
            image: new ol.style.Circle({radius, fill: fill ? new ol.style.Fill({color: fill}) : null,
                stroke: new ol.style.Stroke({color: stroke, width})})
        }));
    }

    function solutionStyle(feature) {
        switch (feature.get("object_type")) {
            case "heat_network": {
                const special = feature.get("laying_method") === "special";
                const dn = Number(feature.get("diameter")) || 0;
                const width = Math.max(3, Math.min(8, 2 + dn / 70));
                return cached(`p:${special}:${width}:${pipeColor(dn)}`, () => new ol.style.Style({
                    stroke: new ol.style.Stroke({color: special ? "#ffffff" : pipeColor(dn), width,
                        lineDash: special ? [8, 6] : undefined})
                }));
            }
            case "heat_network_reconstruction":
                return cached("recon", () => new ol.style.Style({stroke: new ol.style.Stroke({color: "#ff4d6d80", width: 14})}));
            case "tie_in":
                return circle(9, "#ffffff", "#ff4d6d", 3);
            case "heat_chamber":
                return circle(6, "#f0abfc", "#111820", 1.5);
            case "heat_chamber_reconstruction":
                return circle(12, null, "#ff4d6d", 3);
            case "technical_node":
                return circle(3, "#ffffff", "#111820", 1);
            case "unconnected":
                return circle(13, "#ff006e55", "#ff006e", 4);
            case "impact":
                return cached(`i:${feature.get("color")}`, () => new ol.style.Style({
                    stroke: new ol.style.Stroke({color: feature.get("color") + "88", width: 11})}));
            default:
                return null;
        }
    }

    function addLayer(features, zIndex) {
        const source = new ol.source.Vector({wrapX: false,
            features: new ol.format.GeoJSON().readFeatures({type: "FeatureCollection", features}, proj)});
        const layer = new ol.layer.Vector({source, style: solutionStyle, zIndex});
        map.addLayer(layer);
        layers.push(layer);
        return source;
    }

    function clearMap() {
        layers.forEach(layer => map.removeLayer(layer));
        layers = [];
        legend?.remove();
        legend = null;
    }

    function drawLegend() {
        legend = document.createElement("div");
        legend.className = "sol-legend";
        legend.innerHTML = [
            ["line", "#5eead4", "новая труба (толщина — диаметр)"], ["dash", "#ffffff", "специальный проход"],
            ["thick", "#ff4d6d", "реконструкция"], ["ring", "#ff4d6d", "врезка"],
            ["dot", "#f0abfc", "новая камера"], ["ring", "#ff006e", "ОКС без подключения"],
            ["thick", "#fbbf24", "нагрузка на существующую сеть"]
        ].map(([kind, color, text]) => `<div><i class="k-${kind}" style="--c:${color}"></i>${text}</div>`).join("");
        document.getElementById("legend").append(legend);
    }

    function hide() {
        if (map) clearMap();
    }

    function show(variant, box) {
        box.innerHTML = renderDetails(variant);
        if (!map) return;
        clearMap();
        const id = variant.variant_id;

        // Существующие участки, которые новые подключения нагружают, — подсвечены под новой сетью.
        const geometry = new Map();
        for (const f of inputGeojson.features) {
            if (f.properties?.object_type === "heat_network") geometry.set(String(f.properties.id), f.geometry);
        }
        const impacts = variant.impacts.filter(im => geometry.has(im.segment_id)).map(im => ({
            type: "Feature", geometry: geometry.get(im.segment_id),
            properties: {object_type: "impact", color: utilizationColor(im.utilization_after),
                "Участок": im.segment_id, "Диаметр, мм": im.diameter, "Требуется, мм": im.required_diameter,
                "Добавляется, т/ч": im.added_flow_tph,
                "Загрузка": `${Math.round(im.utilization_before * 100)}% → ${Math.round(im.utilization_after * 100)}%`}
        }));
        if (impacts.length) addLayer(impacts, 420);

        const own = plan.geojson.features.filter(f => f.properties.variant_id === id && f.geometry);
        const lines = own.filter(f => f.geometry.type === "LineString");
        const points = own.filter(f => f.geometry.type === "Point");
        addLayer(lines, 600);
        const source = addLayer(points, 650);
        const lost = variant.unconnected.filter(u => u.position).map(u => ({
            type: "Feature", geometry: {type: "Point", coordinates: u.position},
            properties: {object_type: "unconnected", "ОКС": u.oks_id, "Не подключён": u.reason, "Расход, т/ч": u.flow_tph}
        }));
        if (lost.length) addLayer(lost, 700);
        drawLegend();

        // Приблизить карту к новой сети (с отступом под легенду справа).
        const extent = ol.extent.createEmpty();
        layers.forEach(layer => ol.extent.extend(extent, layer.getSource().getExtent()));
        if (!ol.extent.isEmpty(extent)) {
            map.updateSize();
            const legendBox = document.getElementById("legend");
            map.getView().fit(extent, {padding: [40, legendBox.offsetWidth + 40, 42, 64], maxZoom: 18});
        }
        return source;
    }

    // ---------- Панель ----------
    function renderDetails(v) {
        const m = plan.meta;
        const b = v.breakdown;
        const rows = [["Новые трубы", b.construction_cost], ["Новые камеры", b.chamber_construction_cost],
            ["Врезки", b.tie_in_cost], ["Реконструкция труб", b.reconstruction_cost],
            ["Реконструкция камер", b.chamber_reconstruction_cost], ["Штраф за неподключённые ОКС", b.unconnected_penalty]];
        const j = m.joint;
        const list = items => `<ul class="sol-list">${items.map(t => `<li>${esc(t)}</li>`).join("")}</ul>`;

        let html = `<h2 class="sol-title"><span class="sol-rank${v.rank === 1 ? " first" : ""}">${v.rank}</span>${esc(v.name)}</h2>
        <div class="sol-metrics">
            <div><span>Стоимость</span><b>${mln(v.calculated_cost)}</b></div>
            <div><span>Показатель S</span><b>${nf2.format(v.score)}</b></div>
            <div><span>Длина работ</span>${meters(v.length)}</div>
            <div><span>Подключено ОКС</span>${v.connected_oks} из ${m.oks_total}</div>
            <div><span>Врезок / камер</span>${v.tie_in_count} / ${v.new_chamber_count}</div>
            <div><span>Общие трубы</span>${meters(v.shared_length)}</div>
        </div>
        <h3>Почему такой вариант</h3>${list(v.explanation)}
        <h3>Из чего складывается стоимость</h3>
        <table class="sol-table">${rows.map(([name, value]) => `<tr><td>${name}</td><td>${mln(value)}</td></tr>`).join("")}
        <tr class="sumrow"><td>Итого (C)</td><td>${mln(v.calculated_cost)}</td></tr></table>
        <p class="sol-note">S = 0,7 · ${nf1.format(v.calculated_cost / 25e6)} + 0,3 · ${nf1.format(v.length / 100)} = <b>${nf2.format(v.score)}</b>.
        Новая сеть ${meters(v.new_network_length)}, реконструкция ${meters(v.reconstruction_length)}, специальный проход ${meters(v.special_length)}.</p>`;

        if (v.unconnected.length) {
            html += `<h3 class="bad">ОКС без подключения (${v.unconnected.length})</h3><ul class="sol-list">` +
                v.unconnected.map(u => `<li><b>${esc(u.oks_id)}</b> (${nf1.format(u.flow_tph)} т/ч): ${esc(u.reason)}</li>`).join("") + "</ul>";
        }

        html += `<h3>Качество трасс</h3><div class="sol-scroll"><table class="sol-table">
            <tr><th>ОКС</th><th>DN</th><th>Путь</th><th>Прямая</th><th>Извилист.</th></tr>` +
            v.routes.slice().sort((a, c) => c.detour_ratio - a.detour_ratio).map(r =>
                `<tr${r.detour_ratio > 1.8 ? ' class="warn"' : ""}><td>${esc(r.oks_id)}</td><td>${r.diameter}</td><td>${meters(r.length)}</td>
                <td>${meters(r.straight)}</td><td>${nf2.format(r.detour_ratio)}</td></tr>`).join("") + `</table></div>
            <p class="sol-note">Извилистость — длина пути от врезки, делённая на расстояние по прямой. Выше 1,8 подсвечено: обычно на пути запретные зоны.</p>`;

        html += "<h3>Проверки</h3>" + (v.checks.length ? `<div class="warn">${list(v.checks)}</div>`
            : `<p class="sol-note">Нарушений не найдено: зазоры до запретных зон, не более 4 участков у камеры, отсутствие пересечений новых труб и углы пересечения дорог соблюдены.</p>`);

        if (v.impacts.length) {
            html += `<h3>Нагрузка на существующую сеть</h3><div class="sol-scroll"><table class="sol-table">
                <tr><th>Участок</th><th>DN</th><th>+т/ч</th><th>Загрузка</th></tr>` +
                v.impacts.slice().sort((a, c) => c.utilization_after - a.utilization_after).map(im =>
                    `<tr><td>${esc(im.segment_id)}</td><td>${im.diameter}${im.required_diameter > im.diameter ? " → " + im.required_diameter : ""}</td>
                    <td>${nf1.format(im.added_flow_tph)}</td><td><span class="sol-bar"><i style="width:${Math.min(100, Math.round(im.utilization_after * 100))}%;background:${utilizationColor(im.utilization_after)}"></i></span>
                    ${Math.round(im.utilization_before * 100)}→${Math.round(im.utilization_after * 100)}%</td></tr>`).join("") + "</table></div>";
        }

        html += `<h3>Сравнение решений</h3>
        <div class="sol-kpi"><b>${mln(j.saving_rub)}</b><span>экономия от совместных труб: ${nf1.format(j.saving_pct)} % против раздельного подключения
        (${mln(j.separate_cost)}, ${j.separate_tie_ins} врезок)</span></div>
        <div class="sol-kpi"><b>${m.sensitivity.stable ? "Выбор устойчив" : "Выбор зависит от весов"}</b><span>${esc(m.sensitivity.comment)}</span></div>`;
        if (m.notes.length) html += list(m.notes);
        if (m.assumptions.length) html += `<details><summary>Допущения расчёта (${m.assumptions.length})</summary>${list(m.assumptions)}</details>`;
        if (m.diagnostics.length) html += `<details><summary>Диагностика данных (${m.diagnostics.length})</summary>${list(m.diagnostics)}</details>`;
        html += rulesHtml(m.rules);
        return html;
    }

    // «Как это посчитано»: правила технического приложения, которыми пользуется расчёт.
    function rulesHtml(rules) {
        const dn = rules.diameters.map(d => `<tr><td>${d.dn}</td><td>${nf1.format(d.capacity_tph)}</td><td>${nf0.format(d.max_length_m)}</td>
            <td>${nf0.format(d.cost_new)}</td><td>${nf0.format(d.cost_reconstruction)}</td></tr>`).join("");
        const restrictions = rules.restrictions.map(r => `<tr><td>${esc(r.title)}</td><td>${r.forbidden ? "обход" : "спец. проход"}</td>
            <td>${r.min_distance_m}</td><td>${r.forbidden ? "—" : r.k_special}</td></tr>`).join("");
        return `<details><summary>Как это посчитано</summary>
            <p class="sol-note">Выбирается минимальный диаметр, пропускающий расход участка. Врезка стоит ${mln(rules.tie_in_cost)};
            в существующую камеру, если она ближе ${rules.chamber_snap_m} м и у неё меньше ${rules.max_chamber_segments} участков, иначе строится новая камера.
            Штраф за неподключённый ОКС: ${mln(rules.penalty_fixed)} + ${mln(rules.penalty_per_tph)} за каждый т/ч.</p>
            <div class="sol-scroll"><table class="sol-table"><tr><th>DN</th><th>т/ч</th><th>Макс. м</th><th>₽/м</th><th>Реконстр.</th></tr>${dn}</table></div>
            <div class="sol-scroll"><table class="sol-table"><tr><th>Объект</th><th>Правило</th><th>Зазор, м</th><th>Коэфф.</th></tr>${restrictions}</table></div></details>`;
    }

    return {attach, start, show, hide, download};
})();
