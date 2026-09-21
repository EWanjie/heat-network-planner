// Храним исходный File между страницами, не превращая его в строку localStorage.
window.datasetStore = (() => {
    function database() {
        return new Promise((resolve, reject) => {
            const request = indexedDB.open("heatplanner-preview", 1);
            request.onupgradeneeded = () => request.result.createObjectStore("datasets");
            request.onsuccess = () => resolve(request.result);
            request.onerror = () => reject(new Error("Не удалось открыть хранилище браузера."));
        });
    }
    async function save(file, summary) {
        const db = await database();
        const key = sessionStorage.getItem("datasetKey") || crypto.randomUUID();
        try {
            await new Promise((resolve, reject) => {
                const tx = db.transaction("datasets", "readwrite");
                tx.objectStore("datasets").put({file, summary}, key);
                tx.oncomplete = resolve;
                tx.onabort = () => reject(new Error("Недостаточно места или нет доступа к хранилищу браузера."));
                tx.onerror = () => {};
            });
            sessionStorage.setItem("datasetKey", key);
        } finally { db.close(); }
    }
    async function load() {
        const key = sessionStorage.getItem("datasetKey");
        if (!key) return null;
        const db = await database();
        try {
            return await new Promise((resolve, reject) => {
                const request = db.transaction("datasets").objectStore("datasets").get(key);
                request.onsuccess = () => resolve(request.result || null);
                request.onerror = () => reject(new Error("Не удалось открыть сохранённый файл."));
            });
        } finally { db.close(); }
    }
    return {save, load};
})();
