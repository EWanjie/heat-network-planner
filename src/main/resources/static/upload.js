const zone = document.getElementById("dropZone");
const fileInput = document.getElementById("geojsonFile");
const fileInfo = document.getElementById("fileInfo");
const button = document.getElementById("uploadButton");
const status = document.getElementById("status");
let busy = false;
let dragDepth = 0;

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
        status.textContent = "Готовим страницу с картой…";
        await datasetStore.save(file, summary);
        window.location.assign("/results.html");
    } catch (error) {
        showError(error instanceof TypeError ? "Нет связи с сервером. Проверьте, что приложение запущено." : error.message);
        setBusy(false);
    }
}
