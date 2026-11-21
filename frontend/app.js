// SO-ARM 演示控制台前端。无构建步骤，直接由后端以静态文件提供。
"use strict";

const AXIS_LABEL = { x: "前 / 后", y: "左 / 右", z: "上 / 下", pitch: "抬头 / 低头", roll: "腕部旋转", grip: "夹爪 开 / 合" };
const HEAD_LABEL = { pan: "云台 左 / 右", tilt: "云台 上 / 下" };
const JOINT_LABEL = {
  shoulder_pan: "肩部旋转", shoulder_lift: "肩部俯仰", elbow_flex: "肘部",
  wrist_flex: "腕部俯仰", wrist_roll: "腕部旋转", gripper: "夹爪",
};
// 关节条的显示范围（度），仅用于画图
const JOINT_RANGE = {
  shoulder_pan: [-110, 110], shoulder_lift: [-100, 100], elbow_flex: [-97, 97],
  wrist_flex: [-95, 95], wrist_roll: [-160, 165], gripper: [0, 100],
};
const KEY_TEXT = { Comma: ",", Period: ".", ArrowLeft: "←", ArrowRight: "→", ArrowUp: "↑", ArrowDown: "↓" };
const keyText = (code) => KEY_TEXT[code] || code.replace(/^Key|^Digit/, "");

const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];

let meta = null;
let state = null;
let socket = null;
const motionKeys = new Set(); // 会发给后端的运动按键
const pressed = new Set();

// ---------------------------------------------------------------- 请求
async function post(path, body = {}) {
  try {
    const res = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data.detail || `请求失败 (${res.status})`);
    return data;
  } catch (err) {
    toast(err.message);
    return null;
  }
}

let toastTimer = null;
function toast(text) {
  const el = $("#toast");
  el.textContent = text;
  el.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.hidden = true; }, 5000);
}

// ---------------------------------------------------------------- 初始化界面
function keyRows(keymap, labels) {
  // 把 {code: [axis, sign]} 整理成每个轴一行：正方向键在前
  const byAxis = {};
  for (const [code, [axis, sign]] of Object.entries(keymap)) {
    (byAxis[axis] ||= [])[sign > 0 ? 0 : 1] = code;
  }
  return Object.entries(byAxis).map(([axis, codes]) =>
    `<div class="row">${codes.map((c) => `<kbd data-code="${c}">${keyText(c)}</kbd>`).join("")}<span>${labels[axis]}</span></div>`
  ).join("");
}

function addVideo(container, camera) {
  const img = document.createElement("img");
  img.alt = container.dataset.alt;
  img.src = `/stream/${camera}`;
  container.prepend(img);
  return img;
}

function buildUi() {
  for (const panel of $$(".panel[data-arm]")) {
    const arm = panel.dataset.arm;
    const camera = meta.arms[arm]?.camera;
    if (camera) addVideo($(".video", panel), camera);
    panel.dataset.camera = camera || "";
    $(".arm-body", panel).innerHTML = `
      <div class="readout">
        <div><span>前 x (cm)</span><b data-f="x">–</b></div>
        <div><span>左 y (cm)</span><b data-f="y">–</b></div>
        <div><span>高 z (cm)</span><b data-f="z">–</b></div>
        <div><span>俯仰 (°)</span><b data-f="pitch">–</b></div>
      </div>
      <div class="joints">${Object.keys(JOINT_LABEL).map((j) => `
        <div class="joint" data-joint="${j}">
          <span class="name">${JOINT_LABEL[j]}</span>
          <span class="bar"><em></em><i></i></span>
          <span class="val">–</span>
        </div>`).join("")}
      </div>
      <div class="keys">${keyRows(meta.keymap[arm] || {}, AXIS_LABEL)}</div>`;
    Object.keys(meta.keymap[arm] || {}).forEach((c) => motionKeys.add(c));
  }
  const headImg = addVideo($("#head-video"), meta.head_camera);
  headImg.addEventListener("click", onHeadClick);
  $("#head-keys").innerHTML = keyRows(meta.head_keymap, HEAD_LABEL);
  Object.keys(meta.head_keymap).forEach((c) => motionKeys.add(c));
}

// ---------------------------------------------------------------- 渲染状态
function pct(value, [lo, hi]) {
  return `${Math.max(0, Math.min(100, ((value - lo) / (hi - lo)) * 100))}%`;
}

function renderCamera(infoEl, cam) {
  if (!cam) { infoEl.textContent = "未配置"; return; }
  infoEl.classList.toggle("bad", Boolean(cam.error));
  if (cam.error) { infoEl.textContent = `离线：${cam.error}`; return; }
  const detect = cam.detect_error ? "检测不可用" : `手 ${cam.hands}${cam.locked ? " · 已锁定" : ""}`;
  infoEl.textContent = `${cam.fps.toFixed(0)} fps · ${detect}`;
}

function renderArm(panel, arm, cam) {
  const statusEl = $(".arm-status", panel);
  if (!arm) { statusEl.textContent = "未配置"; return; }
  statusEl.className = "arm-status";
  if (arm.lent) {
    statusEl.textContent = "已借给开发调试";
  } else if (!arm.connected) {
    statusEl.textContent = arm.error ? `未连接：${arm.error}` : "未连接";
    statusEl.classList.add("bad");
  } else {
    const calib = arm.calibrated ? `已标定 ${arm.calib_rms_mm} mm` : "未标定外参";
    const ctrl = arm.follow_controller === "smooth" ? "新控制" : "旧控制";
    statusEl.textContent = `${arm.status || "已连接"} · ${ctrl} · ${calib}`;
    statusEl.classList.add("ok");
  }
  panel.classList.toggle("selected", Boolean(arm.fused_selected));
  panel.classList.toggle("disabled", !arm.enabled);
  const toggle = $(".arm-toggle", panel);
  toggle.textContent = arm.enabled ? "停用" : "启用";
  toggle.dataset.next = arm.enabled ? "off" : "on";
  renderCamera($(".cam-info", panel), cam);

  const set = (f, v) => { $(`[data-f="${f}"]`, panel).textContent = v; };
  if (arm.tip) {
    set("x", (arm.tip[0] * 100).toFixed(1));
    set("y", (arm.tip[1] * 100).toFixed(1));
    set("z", (arm.tip[2] * 100).toFixed(1));
    set("pitch", arm.pitch.toFixed(0));
  }
  for (const row of $$(".joint", panel)) {
    const j = row.dataset.joint;
    const now = arm.joints[j];
    if (now === undefined) continue;
    $(".val", row).textContent = now.toFixed(1);
    $("i", row).style.left = pct(now, JOINT_RANGE[j]);
    $("em", row).style.left = pct(arm.target[j] ?? now, JOINT_RANGE[j]);
  }
  $(".keys", panel).classList.toggle("disabled", state.mode !== "keyboard" || state.estop || !arm.enabled);
}

function render() {
  if (!state) return;
  $("#mock-badge").hidden = !state.mock;
  $("#loop-hz").textContent = `控制环 ${state.loop_hz.toFixed(0)} Hz`;
  $("#estop").classList.toggle("active", state.estop);
  $(".estop-label").textContent = state.estop ? "解除急停" : "急停";
  $("#estop-banner").hidden = !state.estop;
  const lent = Object.entries(state.arms).filter(([, a]) => a.lent).map(([n]) => (n === "left" ? "左臂" : "右臂"));
  $("#lent-banner").hidden = lent.length === 0;
  $("#lent-text").textContent = `${lent.join("、")}已借给开发调试，没有参与展示。`;
  for (const btn of $$(".modes button")) {
    btn.classList.toggle("active", btn.dataset.mode === state.mode);
    btn.disabled = state.estop;
  }
  const arms = Object.values(state.arms);
  const anyTorque = arms.some((a) => a.connected && a.torque);
  $("#btn-torque").textContent = anyTorque ? "释放力矩" : "恢复力矩";
  $("#btn-torque").dataset.next = anyTorque ? "off" : "on";
  $("#btn-home").disabled = $("#btn-rest").disabled = state.estop;

  for (const panel of $$(".panel[data-arm]")) {
    renderArm(panel, state.arms[panel.dataset.arm], state.cameras[panel.dataset.camera]);
  }

  renderCamera($("#head-cam-info"), state.cameras[state.head_camera]);
  const head = state.head;
  const headStatus = $("#head-status");
  headStatus.className = "arm-status";
  if (!head) {
    headStatus.textContent = "未配置云台";
  } else if (!head.connected) {
    headStatus.textContent = head.error ? `云台未连接：${head.error}` : "云台未连接";
    headStatus.classList.add("bad");
  } else {
    if (!head.preset) {
      headStatus.textContent = "云台已连接 · 尚未标定";
      headStatus.classList.add("ok");
    } else {
      headStatus.textContent = head.matches_preset ? "云台在标定姿态" : "云台偏离标定姿态，深度引导已暂停";
      headStatus.classList.add(head.matches_preset ? "ok" : "bad");
    }
  }
  if (state.mode === "fused" && !state.depth_guidance_ready) {
    headStatus.className = "arm-status bad";
    headStatus.textContent = "深度引导未生效：未标定，或深度相机姿态与标定时不符";
  }
  $("#head-readout").innerHTML = head && head.pos.pan !== undefined
    ? `<div><span>云台左右 (°)</span><b>${head.pos.pan.toFixed(1)}</b></div>
       <div><span>云台上下 (°)</span><b>${head.pos.tilt.toFixed(1)}</b></div>`
    : "";
  $("#head-keys").classList.toggle("disabled", state.mode !== "keyboard" || state.estop);

  const arm = calibArm();
  const count = state.arms[arm]?.calib_samples ?? 0;
  $("#calib-count").textContent = `${arm === "left" ? "左臂" : "右臂"}已采 ${count} 点`;
  $("#calib-solve").disabled = count < 4;
  const auto = (name) => state.arms[name]?.auto_samples ?? 0;
  $("#auto-count").textContent = `自动样本：左臂 ${auto("left")} 对 · 右臂 ${auto("right")} 对`;
  $("#auto-solve").disabled = auto(arm) < 10;

  for (const kbd of $$(".keys kbd")) kbd.classList.toggle("down", pressed.has(kbd.dataset.code));
}

// ---------------------------------------------------------------- WebSocket 与按键
function sendKeys() {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify({ keys: [...pressed].filter((c) => motionKeys.has(c)) }));
  }
}

function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  socket = new WebSocket(`${proto}://${location.host}/ws`);
  socket.onopen = () => {
    $("#link-dot").classList.add("ok");
    $("#link-text").textContent = "已连接后端";
  };
  socket.onmessage = (ev) => { state = JSON.parse(ev.data); render(); };
  socket.onclose = () => {
    $("#link-dot").classList.remove("ok");
    $("#link-text").textContent = "与后端断开，重连中…";
    setTimeout(connect, 1000);
  };
}

function releaseAll() {
  pressed.clear();
  sendKeys();
  render();
}

const HOTKEYS = {
  Digit0: () => post("/api/mode", { mode: "idle" }),
  Digit1: () => post("/api/mode", { mode: "keyboard" }),
  Digit2: () => post("/api/mode", { mode: "follow" }),
  Digit3: () => post("/api/mode", { mode: "fused" }),
  KeyG: () => post("/api/go", { pose: "home" }),
};

window.addEventListener("keydown", (ev) => {
  if (ev.ctrlKey || ev.metaKey || ev.altKey) return;
  if (ev.code === "Space") {
    ev.preventDefault();
    if (!ev.repeat) post("/api/estop", { active: !(state && state.estop) });
    return;
  }
  if (motionKeys.has(ev.code)) {
    ev.preventDefault();
    if (!pressed.has(ev.code)) { pressed.add(ev.code); sendKeys(); render(); }
    return;
  }
  if (!ev.repeat && HOTKEYS[ev.code]) HOTKEYS[ev.code]();
});
window.addEventListener("keyup", (ev) => {
  if (pressed.delete(ev.code)) { sendKeys(); render(); }
});
// 失焦或切走标签页时浏览器收不到 keyup，必须主动全部松开
window.addEventListener("blur", releaseAll);
document.addEventListener("visibilitychange", () => { if (document.hidden) releaseAll(); });
// 心跳：后端超过 0.35 秒没收到就会当作全部松开
setInterval(sendKeys, 100);

// ---------------------------------------------------------------- 按钮
function calibArm() {
  return $('input[name="calib-arm"]:checked').value;
}

async function onHeadClick(ev) {
  if (!$("#calib-on").checked) return;
  const img = ev.currentTarget;
  const rect = img.getBoundingClientRect();
  // 画面以 contain 方式缩放，需扣掉黑边再换算成归一化坐标
  const scale = Math.min(rect.width / img.naturalWidth, rect.height / img.naturalHeight);
  const w = img.naturalWidth * scale;
  const h = img.naturalHeight * scale;
  const u = (ev.clientX - rect.left - (rect.width - w) / 2) / w;
  const v = (ev.clientY - rect.top - (rect.height - h) / 2) / h;
  if (u < 0 || u > 1 || v < 0 || v > 1) return;
  const res = await post("/api/calib/sample", { arm: calibArm(), u, v });
  if (res) $("#calib-result").textContent = `已记录第 ${res.count} 点（相机系深度 ${res.cam[2].toFixed(3)} m）`;
}

function bindButtons() {
  $("#estop").addEventListener("click", () => post("/api/estop", { active: !(state && state.estop) }));
  for (const btn of $$(".modes button")) {
    btn.addEventListener("click", () => post("/api/mode", { mode: btn.dataset.mode }));
  }
  for (const panel of $$(".panel[data-arm]")) {
    $(".arm-toggle", panel).addEventListener("click", (ev) => {
      post("/api/arm/enable", { arm: panel.dataset.arm, enabled: ev.currentTarget.dataset.next === "on" });
    });
  }
  $("#btn-home").addEventListener("click", () => post("/api/go", { pose: "home" }));
  $("#btn-rest").addEventListener("click", () => post("/api/go", { pose: "rest", release: true }));
  $("#btn-torque").addEventListener("click", (ev) => {
    const off = ev.currentTarget.dataset.next !== "on";
    if (off && !confirm("释放力矩后机械臂会失去支撑并下落。请先用手托住，或改用“收到休息位并释放力矩”。确定释放？")) return;
    post("/api/torque", { active: !off });
  });
  $("#btn-reconnect").addEventListener("click", () => post("/api/reconnect"));
  $("#btn-restore").addEventListener("click", () => post("/api/restore"));
  $("#calib-on").addEventListener("change", (ev) => {
    $("#head-video").classList.toggle("picking", ev.currentTarget.checked);
  });
  $("#calib-undo").addEventListener("click", () => post("/api/calib/undo", { arm: calibArm() }));
  $("#calib-clear").addEventListener("click", () => post("/api/calib/clear", { arm: calibArm() }));
  $("#calib-solve").addEventListener("click", async () => {
    const res = await post("/api/calib/solve", { arm: calibArm() });
    if (res) $("#calib-result").textContent = `已保存：${res.samples} 点，残差 ${(res.rms * 1000).toFixed(1)} mm`;
  });
  $("#auto-solve").addEventListener("click", async () => {
    const res = await post("/api/calib/auto/solve", { arm: calibArm() });
    if (!res) return;
    $("#calib-result").textContent = res.method === "wrist_offset"
      ? `已保存腕部相机安装偏角：偏右 ${res.yaw_right_deg.toFixed(1)}°，偏上 ${res.pitch_up_deg.toFixed(1)}°（采用 ${res.samples} / ${res.collected} 对，散布 ${res.spread_deg.map((v) => v.toFixed(0)).join("° / ")}°）`
      : `已保存：采用 ${res.samples} / ${res.collected} 对，偏差 ${(res.rms * 1000).toFixed(0)} mm`;
  });
  $("#auto-clear").addEventListener("click", () => post("/api/calib/auto/clear", { arm: calibArm() }));
  $("#head-preset").addEventListener("click", () => post("/api/head/preset"));
  // 按钮点击后不要留住焦点，否则空格会再次触发它而不是急停
  document.addEventListener("click", (ev) => { if (ev.target.closest("button")) ev.target.closest("button").blur(); });
}

(async function main() {
  meta = await (await fetch("/api/meta")).json();
  buildUi();
  bindButtons();
  connect();
})();
