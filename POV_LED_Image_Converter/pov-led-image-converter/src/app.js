(function () {
  "use strict";

  const Core = window.POVCore;
  const state = {
    sourceCanvas: document.createElement("canvas"),
    sourceImageData: null,
    sourceName: "built-in-test-image.png",
    sourceAspect: 1,
    conversion: null,
    currentView: "reconstruction",
    selectedSample: 0,
    renderTransform: null,
    mapRect: null,
    drag: null,
    conversionTimer: 0,
    toastTimer: 0,
  };

  const canvas = document.getElementById("previewCanvas");
  const ctx = canvas.getContext("2d", { alpha: false });
  const configInputs = Array.from(document.querySelectorAll("[data-config]"));

  function numberValue(id) {
    return Number(document.getElementById(id).value);
  }

  function readConfig() {
    const config = {};
    configInputs.forEach((input) => {
      if (input.type === "number" || input.type === "range") config[input.dataset.config] = Number(input.value);
      else config[input.dataset.config] = input.value;
    });
    return config;
  }

  function setConfigValue(key, value) {
    const input = document.querySelector(`[data-config="${key}"]`);
    if (!input) return;
    input.value = typeof value === "number" ? String(Number(value.toFixed(4))) : String(value);
  }

  function makeDemoImage() {
    const size = 720;
    const source = state.sourceCanvas;
    source.width = size;
    source.height = size;
    const g = source.getContext("2d");
    g.clearRect(0, 0, size, size);
    g.fillStyle = "#050709";
    g.fillRect(0, 0, size, size);

    const cx = size / 2;
    const cy = size / 2;
    const colors = ["#ff466c", "#ffb52e", "#d6ff3f", "#44e9d5", "#5a8cff", "#bf61ff"];
    g.lineWidth = 42;
    colors.forEach((color, i) => {
      g.strokeStyle = color;
      g.beginPath();
      g.arc(cx, cy, 258 - i * 34, (-.88 + i * .14) * Math.PI, (.04 + i * .13) * Math.PI);
      g.stroke();
    });

    g.save();
    g.translate(cx, cy);
    for (let i = 0; i < 16; i += 1) {
      g.rotate(Math.PI / 8);
      g.fillStyle = colors[i % colors.length];
      g.fillRect(-8, -292, 16, 48);
    }
    g.restore();

    g.fillStyle = "#f5f7f0";
    g.textAlign = "center";
    g.textBaseline = "middle";
    g.font = "900 205px Arial, sans-serif";
    g.fillText("POV", cx, cy + 8);
    g.fillStyle = "#090b0d";
    g.font = "700 29px monospace";
    g.fillText("35 PIXELS / -90°…+90°", cx, cy + 142);
    finishSource("built-in-test-image.png");
  }

  function finishSource(name) {
    state.sourceName = name;
    state.sourceAspect = state.sourceCanvas.width / state.sourceCanvas.height;
    state.sourceImageData = state.sourceCanvas.getContext("2d").getImageData(0, 0, state.sourceCanvas.width, state.sourceCanvas.height);
    document.getElementById("sourceName").textContent = name;
    if (document.getElementById("lockAspect").checked) {
      const width = numberValue("imageWidthCm");
      setConfigValue("imageHeightCm", width / state.sourceAspect);
    }
    scheduleConversion(true);
  }

  function loadImageFile(file) {
    if (!file || !file.type.startsWith("image/")) {
      showToast("Choose a supported image file.", true);
      return;
    }
    const reader = new FileReader();
    reader.onerror = () => showToast("The image could not be read.", true);
    reader.onload = () => {
      const image = new Image();
      image.onerror = () => showToast("The browser could not decode this image.", true);
      image.onload = () => {
        const maxDimension = 2048;
        const scale = Math.min(1, maxDimension / Math.max(image.naturalWidth, image.naturalHeight));
        state.sourceCanvas.width = Math.max(1, Math.round(image.naturalWidth * scale));
        state.sourceCanvas.height = Math.max(1, Math.round(image.naturalHeight * scale));
        const sourceCtx = state.sourceCanvas.getContext("2d");
        sourceCtx.clearRect(0, 0, state.sourceCanvas.width, state.sourceCanvas.height);
        sourceCtx.drawImage(image, 0, 0, state.sourceCanvas.width, state.sourceCanvas.height);
        finishSource(file.name);
        showToast(`Loaded ${file.name}`);
      };
      image.src = reader.result;
    };
    reader.readAsDataURL(file);
  }

  function scheduleConversion(immediate) {
    window.clearTimeout(state.conversionTimer);
    if (immediate) runConversion();
    else state.conversionTimer = window.setTimeout(runConversion, 70);
  }

  function runConversion() {
    if (!state.sourceImageData) return;
    try {
      const selectedAngle = state.conversion
        ? Core.angleForSample(state.conversion.config, state.selectedSample)
        : 0;
      state.conversion = Core.convertImage(state.sourceImageData, readConfig());
      state.selectedSample = Core.sampleForAngle(state.conversion.config, selectedAngle);
      updateReadouts();
      drawPreview();
      renderLedStrip();
    } catch (error) {
      showToast(error.message || "Conversion failed.", true);
    }
  }

  function formatBytes(bytes) {
    if (bytes < 1024) return `${bytes} B`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KiB`;
    return `${(bytes / (1024 * 1024)).toFixed(2)} MiB`;
  }

  function formatRate(bytesPerSecond) {
    if (bytesPerSecond < 1024) return `${bytesPerSecond.toFixed(0)} B/s`;
    if (bytesPerSecond < 1024 * 1024) return `${(bytesPerSecond / 1024).toFixed(0)} KiB/s`;
    return `${(bytesPerSecond / (1024 * 1024)).toFixed(2)} MiB/s`;
  }

  function updateReadouts() {
    const conversion = state.conversion;
    const config = conversion.config;
    document.getElementById("frameCount").textContent = config.sampleCount.toLocaleString();
    document.getElementById("actualStep").textContent = `${config.actualStepDeg.toFixed(6)}°`;
    document.getElementById("sampleCount").textContent = conversion.totalLedSamples.toLocaleString();
    document.getElementById("payloadSize").textContent = formatBytes(Core.HEADER_BYTES + conversion.payload.length);
    document.getElementById("litPercent").textContent = `${(100 * conversion.litSamples / Math.max(1, conversion.totalLedSamples)).toFixed(1)}%`;
    document.getElementById("ledPitchReadout").textContent = `${config.ledPitchCm.toFixed(3)} cm`;
    document.getElementById("rgbBrightnessOutput").textContent = `${Math.round(config.rgbBrightnessPercent)}%`;
    document.getElementById("outerLedLabel").textContent = `LED ${config.ledCount - 1} · farthest`;
    document.getElementById("angleSlider").max = String(config.sampleCount - 1);
    document.getElementById("angleSlider").value = String(state.selectedSample);

    const adjustment = document.getElementById("stepAdjustment");
    if (Math.abs(config.requestedStepDeg - config.actualStepDeg) > 0.000001) {
      adjustment.hidden = false;
      adjustment.textContent = `Adjusted ${config.requestedStepDeg}° to ${config.actualStepDeg.toFixed(6)}° so the 180° sweep has whole intervals and includes both endpoints.`;
    } else adjustment.hidden = true;

    updateRuntimeEstimate();
    updateAngleLabel();
  }

  function calculateRuntimeEstimate(config) {
    const cyclesPerSecond = Math.max(0, numberValue("waveCyclesPerSecond") || 0);
    const waveAngleDeg = Math.min(90, Math.max(0, numberValue("waveAngleDeg") || 0));
    const spiMhz = Math.max(.1, numberValue("spiMhz") || .1);
    const updatesPerSecond = 4 * waveAngleDeg * cyclesPerSecond / config.actualStepDeg;
    const peakUpdatesPerSecond = 2 * Math.PI * waveAngleDeg * cyclesPerSecond / config.actualStepDeg;
    const peakAngularRateDps = 2 * Math.PI * waveAngleDeg * cyclesPerSecond;
    const peakLedBytesPerSecond = peakUpdatesPerSecond * config.frameBytes;
    const busUse = peakLedBytesPerSecond * 8 / (spiMhz * 1000000) * 100;
    return {
      cyclesPerSecond,
      waveAngleDeg,
      spiMhz,
      updatesPerSecond,
      peakUpdatesPerSecond,
      peakAngularRateDps,
      peakLedBytesPerSecond,
      busUse,
    };
  }

  function updateRuntimeEstimate() {
    if (!state.conversion) return;
    const estimate = calculateRuntimeEstimate(state.conversion.config);
    const {
      updatesPerSecond,
      peakUpdatesPerSecond,
      peakAngularRateDps,
      peakLedBytesPerSecond,
      busUse,
    } = estimate;
    document.getElementById("updatesPerSecond").textContent = updatesPerSecond.toLocaleString(undefined, { maximumFractionDigits: 0 });
    document.getElementById("peakUpdatesPerSecond").textContent = peakUpdatesPerSecond.toLocaleString(undefined, { maximumFractionDigits: 0 });
    document.getElementById("peakLedDataRate").textContent = formatRate(peakLedBytesPerSecond);
    document.getElementById("startupRead").textContent = formatBytes(Core.HEADER_BYTES + state.conversion.payload.length);
    document.getElementById("peakAngularRate").textContent = `${peakAngularRateDps.toLocaleString(undefined, { maximumFractionDigits: 0 })} °/s`;
    document.getElementById("busUse").textContent = `${busUse.toFixed(1)}%`;
    const bar = document.getElementById("performanceBar");
    bar.style.width = `${Math.min(100, busUse)}%`;
    const status = document.getElementById("performanceStatus");
    if (peakAngularRateDps > 4000) {
      bar.style.background = "var(--danger)";
      status.textContent = "This wave exceeds the firmware's ±4000 °/s gyro range. Reduce cycles/second or wave angle.";
      status.style.color = "var(--danger)";
    } else if (busUse > 100) {
      bar.style.background = "var(--danger)";
      status.textContent = "The selected SPI clock cannot transmit the peak angle-frame rate.";
      status.style.color = "var(--danger)";
    } else if (peakUpdatesPerSecond > 1920) {
      bar.style.background = "var(--amber)";
      status.textContent = "The LED bus fits, but the 1,920 Hz orientation loop may skip some angle bins near the middle of the wave.";
      status.style.color = "var(--amber)";
    } else if (busUse > 80) {
      bar.style.background = "var(--amber)";
      status.textContent = "Timing fits with little SPI margin.";
      status.style.color = "var(--amber)";
    } else {
      bar.style.background = "var(--lime)";
      status.textContent = "The 1,920 Hz orientation loop and LED bus both cover this motion estimate.";
      status.style.color = "var(--muted)";
    }
  }

  function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 1.5);
    const width = Math.max(320, Math.round(rect.width * dpr));
    const height = Math.max(300, Math.round(rect.height * dpr));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
  }

  function physicalExtent(config) {
    const r0 = Math.abs(Core.radiusForLed(config, 0));
    const rn = Math.abs(Core.radiusForLed(config, config.ledCount - 1));
    const imageRadius = Math.hypot(config.imageCenterXCm, config.imageCenterYCm) + Math.hypot(config.imageWidthCm, config.imageHeightCm) / 2;
    return Math.max(3, r0, rn, imageRadius) * 1.13;
  }

  function setupPhysicalTransform(config) {
    const extent = physicalExtent(config);
    const scale = Math.min(canvas.width, canvas.height) * .43 / extent;
    const transform = { cx: canvas.width / 2, cy: canvas.height / 2, scale, extent };
    state.renderTransform = transform;
    return transform;
  }

  function toCanvas(transform, x, y) {
    return { x: transform.cx + x * transform.scale, y: transform.cy - y * transform.scale };
  }

  function drawGrid(config, transform) {
    ctx.fillStyle = "#07090b";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.save();
    ctx.lineWidth = Math.max(1, canvas.width / 1200);
    ctx.strokeStyle = "#1a2025";
    const ringStep = transform.extent > 60 ? 10 : 5;
    for (let radius = ringStep; radius < transform.extent; radius += ringStep) {
      ctx.beginPath();
      ctx.arc(transform.cx, transform.cy, radius * transform.scale, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.setLineDash([5, 7]);
    ctx.strokeStyle = "#293139";
    ctx.beginPath();
    ctx.moveTo(0, transform.cy);
    ctx.lineTo(canvas.width, transform.cy);
    ctx.moveTo(transform.cx, 0);
    ctx.lineTo(transform.cx, canvas.height);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.font = `${Math.max(9, canvas.width / 95)}px monospace`;
    ctx.fillStyle = "#535e66";
    ctx.fillText("+X", canvas.width - 32, transform.cy - 8);
    ctx.fillText("+Y", transform.cx + 8, 17);
    ctx.restore();
  }

  function drawSource(config, transform, opacity, boundsOnly) {
    if (!document.getElementById("showSource").checked && !boundsOnly) return;
    const center = toCanvas(transform, config.imageCenterXCm, config.imageCenterYCm);
    const width = config.imageWidthCm * transform.scale;
    const height = config.imageHeightCm * transform.scale;
    ctx.save();
    ctx.translate(center.x, center.y);
    ctx.rotate(config.imageRotationDeg * Math.PI / 180);
    if (!boundsOnly) {
      ctx.globalAlpha = opacity;
      ctx.drawImage(state.sourceCanvas, -width / 2, -height / 2, width, height);
    }
    ctx.globalAlpha = 1;
    ctx.setLineDash([7, 6]);
    ctx.strokeStyle = "#ffb02e";
    ctx.lineWidth = Math.max(1, canvas.width / 1000);
    ctx.strokeRect(-width / 2, -height / 2, width, height);
    ctx.setLineDash([]);
    const handle = Math.max(4, canvas.width / 220);
    ctx.fillStyle = "#ffb02e";
    [[-width/2,-height/2],[width/2,-height/2],[width/2,height/2],[-width/2,height/2]].forEach(([x,y]) => ctx.fillRect(x-handle/2,y-handle/2,handle,handle));
    ctx.restore();
  }

  function drawRotationPaths(config, transform) {
    ctx.save();
    ctx.strokeStyle = "rgba(94,231,242,.17)";
    ctx.lineWidth = Math.max(1, canvas.width / 1300);
    const stride = Math.max(1, Math.ceil(config.ledCount / 12));
    for (let led = 0; led < config.ledCount; led += stride) {
      ctx.beginPath();
      for (let sample = 0; sample < config.sampleCount; sample += 1) {
        const world = Core.ledWorldPosition(config, sample, led);
        const point = toCanvas(transform, world.x, world.y);
        if (sample === 0) ctx.moveTo(point.x, point.y);
        else ctx.lineTo(point.x, point.y);
      }
      ctx.stroke();
    }
    ctx.restore();
  }

  function drawConvertedPoints(conversion, transform) {
    const config = conversion.config;
    const sampleStride = Math.max(1, Math.ceil(config.sampleCount / 1440));
    const pointSize = Math.max(1.25, Math.min(3.5, transform.scale * config.ledPitchCm * .34));
    ctx.save();
    ctx.globalCompositeOperation = "lighter";
    for (let sample = 0; sample < config.sampleCount; sample += sampleStride) {
      for (let led = 0; led < config.ledCount; led += 1) {
        const rgb = Core.frameRgb(conversion, sample, led);
        if (!(rgb[0] || rgb[1] || rgb[2])) continue;
        const world = Core.ledWorldPosition(config, sample, led);
        const point = toCanvas(transform, world.x, world.y);
        ctx.fillStyle = `rgb(${rgb[0]},${rgb[1]},${rgb[2]})`;
        ctx.fillRect(point.x - pointSize / 2, point.y - pointSize / 2, pointSize, pointSize);
      }
    }
    ctx.restore();
  }

  function drawSelectedRay(config, transform) {
    const first = Core.ledWorldPosition(config, state.selectedSample, 0);
    const last = Core.ledWorldPosition(config, state.selectedSample, config.ledCount - 1);
    const a = toCanvas(transform, first.x, first.y);
    const b = toCanvas(transform, last.x, last.y);
    ctx.save();
    ctx.strokeStyle = "rgba(255,255,255,.72)";
    ctx.lineWidth = Math.max(1.2, canvas.width / 800);
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
    const dot = Math.max(2.3, Math.min(6, transform.scale * config.ledPitchCm * .45));
    for (let led = 0; led < config.ledCount; led += 1) {
      const world = Core.ledWorldPosition(config, state.selectedSample, led);
      const p = toCanvas(transform, world.x, world.y);
      const rgb = Core.frameRgb(state.conversion, state.selectedSample, led);
      ctx.fillStyle = `rgb(${rgb[0]},${rgb[1]},${rgb[2]})`;
      ctx.beginPath();
      ctx.arc(p.x, p.y, dot, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "rgba(255,255,255,.5)";
      ctx.stroke();
    }
    ctx.fillStyle = "#c8ff3d";
    ctx.beginPath();
    ctx.arc(transform.cx, transform.cy, Math.max(4, canvas.width / 170), 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }

  function drawMap(conversion) {
    state.renderTransform = null;
    ctx.fillStyle = "#07090b";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    const config = conversion.config;
    const mapWidth = Math.min(config.sampleCount, 2400);
    const mapHeight = Math.min(config.ledCount, 640);
    const offscreen = document.createElement("canvas");
    offscreen.width = mapWidth;
    offscreen.height = mapHeight;
    const offCtx = offscreen.getContext("2d");
    const pixels = offCtx.createImageData(mapWidth, mapHeight);
    for (let x = 0; x < mapWidth; x += 1) {
      const sample = Math.min(config.sampleCount - 1, Math.floor(x / mapWidth * config.sampleCount));
      for (let y = 0; y < mapHeight; y += 1) {
        const ledFromTop = Math.min(config.ledCount - 1, Math.floor(y / mapHeight * config.ledCount));
        const led = config.ledCount - 1 - ledFromTop;
        const rgb = Core.frameRgb(conversion, sample, led);
        const offset = (y * mapWidth + x) * 4;
        pixels.data[offset] = rgb[0];
        pixels.data[offset + 1] = rgb[1];
        pixels.data[offset + 2] = rgb[2];
        pixels.data[offset + 3] = 255;
      }
    }
    offCtx.putImageData(pixels, 0, 0);
    const marginX = Math.max(42, canvas.width * .07);
    const marginY = Math.max(38, canvas.height * .09);
    const rect = { x: marginX, y: marginY, width: canvas.width - 2 * marginX, height: canvas.height - 2 * marginY };
    state.mapRect = rect;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(offscreen, rect.x, rect.y, rect.width, rect.height);
    ctx.strokeStyle = "#46515a";
    ctx.strokeRect(rect.x, rect.y, rect.width, rect.height);
    const selectedX = rect.x + state.selectedSample / (config.sampleCount - 1) * rect.width;
    ctx.strokeStyle = "#f4f7ef";
    ctx.lineWidth = Math.max(1, canvas.width / 800);
    ctx.beginPath();
    ctx.moveTo(selectedX, rect.y - 8);
    ctx.lineTo(selectedX, rect.y + rect.height + 8);
    ctx.stroke();
    ctx.fillStyle = "#89939b";
    ctx.font = `${Math.max(9, canvas.width / 90)}px monospace`;
    ctx.textAlign = "center";
    [-90, -45, 0, 45, 90].forEach((angle) => {
      const x = rect.x + (angle - config.minAngleDeg) / (config.maxAngleDeg - config.minAngleDeg) * rect.width;
      ctx.fillText(`${angle}°`, x, rect.y + rect.height + 24);
    });
    ctx.save();
    ctx.translate(rect.x - 20, rect.y + rect.height / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText(`LED 0 → LED ${config.ledCount - 1}`, 0, 0);
    ctx.restore();
    document.getElementById("canvasMessage").textContent = "Click the map to inspect an angle";
  }

  function drawPreview() {
    if (!state.conversion) return;
    resizeCanvas();
    state.mapRect = null;
    const conversion = state.conversion;
    const config = conversion.config;
    if (state.currentView === "map") {
      drawMap(conversion);
      return;
    }
    const transform = setupPhysicalTransform(config);
    drawGrid(config, transform);
    drawRotationPaths(config, transform);
    if (state.currentView === "geometry") {
      drawSource(config, transform, .85, false);
      drawSelectedRay(config, transform);
      document.getElementById("canvasMessage").textContent = "Drag image to position it";
      return;
    }
    if (document.getElementById("showSource").checked) drawSource(config, transform, .14, false);
    else drawSource(config, transform, 0, true);
    drawConvertedPoints(conversion, transform);
    drawSelectedRay(config, transform);
    document.getElementById("canvasMessage").textContent = "Drag image to position it";
  }

  function renderLedStrip() {
    if (!state.conversion) return;
    const config = state.conversion.config;
    const holder = document.getElementById("ledStrip");
    holder.replaceChildren();
    holder.style.gridTemplateColumns = `repeat(${config.ledCount}, minmax(3px, 1fr))`;
    for (let led = 0; led < config.ledCount; led += 1) {
      const rgb = Core.frameRgb(state.conversion, state.selectedSample, led);
      const hex = `#${rgb.map((v) => v.toString(16).padStart(2, "0")).join("").toUpperCase()}`;
      const chip = document.createElement("span");
      chip.className = "led-chip";
      chip.dataset.led = String(led);
      chip.style.background = `rgb(${rgb[0]},${rgb[1]},${rgb[2]})`;
      chip.title = `LED ${led} · RGB(${rgb.join(", ")}) · ${hex} · radius ${Core.radiusForLed(config, led).toFixed(3)} cm`;
      holder.appendChild(chip);
    }
    updateAngleLabel();
  }

  function updateAngleLabel() {
    if (!state.conversion) return;
    const angle = Core.angleForSample(state.conversion.config, state.selectedSample);
    document.getElementById("selectedAngleLabel").textContent = `${angle.toFixed(3)}°`;
    document.getElementById("angleSlider").value = String(state.selectedSample);
  }

  function selectSample(sample) {
    if (!state.conversion) return;
    const count = state.conversion.config.sampleCount;
    state.selectedSample = Math.min(count - 1, Math.max(0, Math.round(sample)));
    renderLedStrip();
    drawPreview();
  }

  function pointerToCanvas(event) {
    const rect = canvas.getBoundingClientRect();
    return {
      x: (event.clientX - rect.left) * canvas.width / rect.width,
      y: (event.clientY - rect.top) * canvas.height / rect.height,
    };
  }

  function canvasToWorld(point) {
    const t = state.renderTransform;
    return { x: (point.x - t.cx) / t.scale, y: (t.cy - point.y) / t.scale };
  }

  function pointInsideImage(world, config) {
    const uv = Core.worldToImageUv(config, world.x, world.y);
    return uv.u >= 0 && uv.u <= 1 && uv.v >= 0 && uv.v <= 1;
  }

  function handleCanvasPointerDown(event) {
    if (!state.conversion) return;
    const point = pointerToCanvas(event);
    if (state.currentView === "map" && state.mapRect) {
      const rect = state.mapRect;
      if (point.x >= rect.x && point.x <= rect.x + rect.width) {
        selectSample((point.x - rect.x) / rect.width * (state.conversion.config.sampleCount - 1));
      }
      return;
    }
    if (!state.renderTransform) return;
    const world = canvasToWorld(point);
    const config = state.conversion.config;
    if (!pointInsideImage(world, config)) return;
    state.drag = { startWorld: world, centerX: config.imageCenterXCm, centerY: config.imageCenterYCm };
    canvas.classList.add("dragging");
    canvas.setPointerCapture(event.pointerId);
  }

  function handleCanvasPointerMove(event) {
    if (!state.drag || !state.renderTransform) return;
    const world = canvasToWorld(pointerToCanvas(event));
    setConfigValue("imageCenterXCm", state.drag.centerX + world.x - state.drag.startWorld.x);
    setConfigValue("imageCenterYCm", state.drag.centerY + world.y - state.drag.startWorld.y);
    scheduleConversion(false);
  }

  function handleCanvasPointerUp(event) {
    if (!state.drag) return;
    state.drag = null;
    canvas.classList.remove("dragging");
    if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
  }

  function downloadBlob(blob, fileName) {
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url;
    link.download = fileName;
    document.body.appendChild(link);
    link.click();
    link.remove();
    window.setTimeout(() => URL.revokeObjectURL(url), 1000);
  }

  function downloadPov() {
    if (!state.conversion) return;
    const bytes = Core.buildWandBinary(state.conversion);
    downloadBlob(new Blob([bytes], { type: "application/octet-stream" }), "WAND.POV");
    showToast(`Exported ${formatBytes(bytes.length)} WAND1 binary`);
  }

  function downloadJson() {
    if (!state.conversion) return;
    const estimate = calculateRuntimeEstimate(state.conversion.config);
    const data = Core.buildJsonExport(state.conversion, state.sourceName, {
      cycles_per_second: estimate.cyclesPerSecond,
      maximum_deviation_deg: estimate.waveAngleDeg,
      average_frame_changes_per_second: estimate.updatesPerSecond,
      peak_frame_changes_per_second: estimate.peakUpdatesPerSecond,
      peak_angular_rate_deg_per_second: estimate.peakAngularRateDps,
      peak_led_bytes_per_second: estimate.peakLedBytesPerSecond,
      spi_clock_mhz: estimate.spiMhz,
      peak_spi_bus_use_percent: estimate.busUse,
      firmware_orientation_rate_hz: 1920,
    });
    const text = JSON.stringify(data, null, 2);
    downloadBlob(new Blob([text], { type: "application/json" }), "WAND.json");
    showToast("Exported the human-readable SK9822 command translation");
  }

  function showToast(message, error) {
    const toast = document.getElementById("toast");
    toast.textContent = message;
    toast.style.borderColor = error ? "var(--danger)" : "var(--lime)";
    toast.classList.add("show");
    window.clearTimeout(state.toastTimer);
    state.toastTimer = window.setTimeout(() => toast.classList.remove("show"), 2600);
  }

  configInputs.forEach((input) => {
    input.addEventListener("input", () => {
      if (document.getElementById("lockAspect").checked && state.sourceAspect) {
        if (input.id === "imageWidthCm") setConfigValue("imageHeightCm", numberValue("imageWidthCm") / state.sourceAspect);
        if (input.id === "imageHeightCm") setConfigValue("imageWidthCm", numberValue("imageHeightCm") * state.sourceAspect);
      }
      scheduleConversion(false);
    });
    input.addEventListener("change", () => scheduleConversion(false));
  });

  document.getElementById("waveCyclesPerSecond").addEventListener("input", updateRuntimeEstimate);
  document.getElementById("waveAngleDeg").addEventListener("input", updateRuntimeEstimate);
  document.getElementById("spiMhz").addEventListener("input", updateRuntimeEstimate);
  document.getElementById("showSource").addEventListener("change", drawPreview);
  document.getElementById("loadDemoButton").addEventListener("click", makeDemoImage);
  document.getElementById("imageInput").addEventListener("change", (event) => loadImageFile(event.target.files[0]));
  document.getElementById("angleSlider").addEventListener("input", (event) => selectSample(Number(event.target.value)));
  document.getElementById("downloadPov").addEventListener("click", downloadPov);
  document.getElementById("downloadJson").addEventListener("click", downloadJson);

  const dropZone = document.getElementById("dropZone");
  ["dragenter", "dragover"].forEach((type) => dropZone.addEventListener(type, (event) => {
    event.preventDefault();
    dropZone.classList.add("dragging");
  }));
  ["dragleave", "drop"].forEach((type) => dropZone.addEventListener(type, (event) => {
    event.preventDefault();
    dropZone.classList.remove("dragging");
  }));
  dropZone.addEventListener("drop", (event) => loadImageFile(event.dataTransfer.files[0]));

  document.querySelectorAll(".view-tab").forEach((button) => {
    button.addEventListener("click", () => {
      document.querySelectorAll(".view-tab").forEach((item) => item.classList.remove("active"));
      button.classList.add("active");
      state.currentView = button.dataset.view;
      drawPreview();
    });
  });

  canvas.addEventListener("pointerdown", handleCanvasPointerDown);
  canvas.addEventListener("pointermove", handleCanvasPointerMove);
  canvas.addEventListener("pointerup", handleCanvasPointerUp);
  canvas.addEventListener("pointercancel", handleCanvasPointerUp);

  if (window.ResizeObserver) {
    const observer = new ResizeObserver(() => drawPreview());
    observer.observe(document.getElementById("canvasShell"));
  } else window.addEventListener("resize", drawPreview);

  makeDemoImage();
})();
