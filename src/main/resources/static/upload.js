const zone = document.getElementById("dropZone");
const fileInput = document.getElementById("geojsonFile");
const fileInfo = document.getElementById("fileInfo");
const button = document.getElementById("uploadButton");
const status = document.getElementById("status");
let busy = false;
let dragDepth = 0;
let selectedDataset = null;
let activeCalculation = null;
const calculationPanel = document.getElementById("calculationPanel");
function cancelCalculation() {
    if (!activeCalculation) return;
    const current = activeCalculation;
    current.cancelled = true;
    fetch(`/api/plan/${current.id}`, {method: "DELETE", keepalive: true}).catch(() => {});
    current.controller.abort();
}
document.getElementById("cancelCalculation").addEventListener("click", cancelCalculation);
document.getElementById("retryCalculation").addEventListener("click", () => calculate(selectedDataset));
document.getElementById("chooseAnotherFile").addEventListener("click", () => {
    calculationPanel.hidden = true;
    document.getElementById("uploadPanel").hidden = false;
    document.querySelector(".upload-heading").hidden = false;
    selectedDataset = null; setBusy(false); status.textContent = ""; fileInfo.hidden = true;
});
window.addEventListener("pagehide", cancelCalculation);
window.addEventListener("beforeunload", event => {
    if (activeCalculation) {event.preventDefault(); event.returnValue = "Данные могут не сохраниться";}
});

async function calculate(data) {
    if (!data || activeCalculation) return;
    setBusy(true);
    document.getElementById("uploadPanel").hidden = true;
    document.querySelector(".upload-heading").hidden = true;
    calculationPanel.hidden = false;
    const title = document.getElementById("calculationTitle"), message = document.getElementById("calculationStatus");
    const spinner = document.getElementById("calculationSpinner"), cancel = document.getElementById("cancelCalculation");
    const retry = document.getElementById("retryCalculation"), another = document.getElementById("chooseAnotherFile");
    title.textContent = "Строим схемы подключения";
    message.textContent = "Сервер рассчитывает маршруты по загруженным данным. Обычно это занимает до 30 секунд; проверка результата может потребовать ещё немного времени.";
    document.getElementById("calculationFile").textContent = data.file.name;
    spinner.hidden = false; cancel.hidden = false; retry.hidden = true; another.hidden = true;
    const current = {id: crypto.randomUUID(), controller: new AbortController(), cancelled: false};
    activeCalculation = current;
    const started = Date.now();
    const updateTime = () => document.getElementById("calculationElapsed").textContent = `Прошло: ${Math.floor((Date.now() - started) / 1000)} с`;
    updateTime(); const timer = setInterval(updateTime, 1000);
    const timeout = setTimeout(() => {
        current.timedOut = true;
        fetch(`/api/plan/${current.id}`, {method: "DELETE", keepalive: true}).catch(() => {});
        current.controller.abort();
    }, 60000);
    try {
        const dataset = JSON.parse(await data.file.text());
        if (current.cancelled) throw new DOMException("Cancelled", "AbortError");
        const response = await fetch("/api/plan", {method: "POST", headers: {"Content-Type": "application/json"}, signal: current.controller.signal,
            body: JSON.stringify({requestId: current.id, dataset, osm: {required: false, status: "disabled"},
                options: {maxTimeMillis: 30000, totalSearchBudget: 30000}})});
        if (!response.ok) {
            const body = await response.json().catch(() => ({}));
            throw new Error(body.error || `Ошибка сервера: HTTP ${response.status}`);
        }
        const result = await response.json();
        if (!Array.isArray(result.variants)) throw new Error("Сервер вернул некорректный результат расчёта.");
        if (!result.variants.length) {
            title.textContent = "Решение пока не найдено";
            message.textContent = "За отведённое время не удалось построить проверенную схему. Это не означает, что подключение невозможно. Можно повторить расчёт или выбрать другой файл.";
            return;
        }
        if (current.cancelled || current.timedOut) throw new DOMException("Cancelled", "AbortError");
        await datasetStore.save(data.file, data.summary, result);
        if (current.cancelled || current.timedOut) throw new DOMException("Cancelled", "AbortError");
        activeCalculation = null;
        window.location.assign("/results.html");
    } catch (error) {
        title.textContent = current.cancelled ? "Расчёт отменён" : "Не удалось завершить расчёт";
        message.textContent = current.cancelled ? "Можно повторить расчёт или выбрать другой файл." : current.timedOut ?
            "Сервер не вернул ответ за минуту. Попробуйте повторить расчёт." : error instanceof TypeError ?
                "Нет связи с сервером. Проверьте, что приложение запущено, и повторите расчёт." : error.message;
    } finally {
        clearInterval(timer); clearTimeout(timeout); activeCalculation = null; setBusy(false);
        spinner.hidden = true; cancel.hidden = true; retry.hidden = false; another.hidden = false;
    }
}

function showError(message) {
    status.classList.add("error");
    status.textContent = message;
}
function setBusy(value) {
    busy = value;
    button.disabled = value;
    fileInput.disabled = value;
    button.textContent = value ? "Загрузка…" : "Загрузить файл";
    zone.setAttribute("aria-busy", String(value));
}
function chooseFile() {
    if (busy) return;
    fileInput.value = ""; // Позволяет повторно выбрать тот же файл после ошибки.
    fileInput.click();
}
button.addEventListener("click", event => {
    event.stopPropagation();
    chooseFile();
});
zone.addEventListener("click", event => {
    if (event.target !== fileInput) chooseFile();
});
fileInput.addEventListener("change", () => acceptFiles(fileInput.files));

function hasFiles(event) {
    return Array.from(event.dataTransfer?.types || []).includes("Files");
}
// Не позволяем браузеру открыть брошенный файл вместо приложения.
document.addEventListener("dragover", event => { if (hasFiles(event)) event.preventDefault(); });
document.addEventListener("drop", event => { if (hasFiles(event)) event.preventDefault(); });
zone.addEventListener("dragenter", event => {
    if (!hasFiles(event)) return;
    event.preventDefault();
    if (!busy) { dragDepth++; zone.classList.add("is-dragging"); }
});
zone.addEventListener("dragover", event => {
    if (!hasFiles(event)) return;
    event.preventDefault();
    event.dataTransfer.dropEffect = busy ? "none" : "copy";
});
zone.addEventListener("dragleave", () => {
    dragDepth = Math.max(0, dragDepth - 1);
    if (!dragDepth) zone.classList.remove("is-dragging");
});
zone.addEventListener("drop", event => {
    if (!hasFiles(event)) return;
    event.preventDefault();
    dragDepth = 0;
    zone.classList.remove("is-dragging");
    if (!busy) acceptFiles(event.dataTransfer.files);
});
window.addEventListener("pageshow", () => {
    if (activeCalculation) return;
    calculationPanel.hidden = true;
    document.getElementById("uploadPanel").hidden = false;
    document.querySelector(".upload-heading").hidden = false;
    selectedDataset = null;
    setBusy(false);
    dragDepth = 0;
    zone.classList.remove("is-dragging");
    status.textContent = "";
    status.classList.remove("error");
    fileInfo.hidden = true;
    fileInput.value = "";
});

function acceptFiles(files) {
    if (busy || files.length === 0) return;
    fileInfo.hidden = true;
    status.classList.remove("error");
    status.textContent = "";
    if (files.length !== 1) { showError("Пожалуйста, выберите или перетащите только один файл."); return; }
    const file = files[0];
    if (!/\.(geojson|json)$/i.test(file.name)) { showError("Выберите файл с расширением .geojson или .json."); return; }
    if (file.size === 0) { showError("Выбранный файл пуст."); return; }
    fileInfo.textContent = `${file.name} · ${(file.size / 1024).toLocaleString("ru-RU", {maximumFractionDigits: 1})} КиБ`;
    fileInfo.hidden = false;
    upload(file);
}

async function upload(file) {
    setBusy(true);
    status.textContent = "Загружаем файл и проверяем данные…";
    try {
        const body = new FormData();
        body.append("file", file);
        const response = await fetch("/api/upload", {method: "POST", body});
        if (!response.ok) {
            const text = await response.text();
            throw new Error(response.status === 413 ? "Файл превышает допустимый размер загрузки." :
                response.status >= 500 ? "Сервер не смог обработать файл. Попробуйте ещё раз." : text);
        }
        const summary = await response.json();
        selectedDataset = {file, summary};
        await calculate(selectedDataset);
    } catch (error) {
        showError(error instanceof TypeError ? "Нет связи с сервером. Проверьте, что приложение запущено." : error.message);
        setBusy(false);
    }
}
