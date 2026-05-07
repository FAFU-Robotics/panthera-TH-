// ============================================================================
//  app.js  —  Panthera-HT Web Control 前端
//
//  纯 vanilla JS, 无构建步骤. 所有交互通过 fetch() 调 Flask /api/*.
// ============================================================================

// ---------- 工具 ----------
const $  = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => Array.from(root.querySelectorAll(sel));

const fmt = (v, prec = 4) =>
    (v === null || v === undefined || isNaN(v)) ? '—' :
    (Number(v).toFixed(prec));

async function api(path, opts = {}) {
    const init = {
        method: opts.method || 'GET',
        headers: { 'Content-Type': 'application/json' },
    };
    if (opts.body !== undefined) init.body = JSON.stringify(opts.body);
    let resp, data;
    try {
        resp = await fetch(path, init);
        data = await resp.json();
    } catch (e) {
        log('error', `${opts.method || 'GET'} ${path} 网络错误: ${e}`);
        return { ok: false, message: String(e) };
    }
    if (!resp.ok || data.ok === false) {
        log('error', `${path}: ${data.message || resp.statusText}`);
    }
    return data;
}

// ---------- 日志 ----------
const logArea = $('#logArea');
function log(level, msg) {
    const line = document.createElement('div');
    line.className = `log-line l-${level}`;
    const t = new Date().toTimeString().slice(0, 8);
    line.innerHTML = `<span class="log-time">${t}</span>${escapeHtml(msg)}`;
    logArea.appendChild(line);
    logArea.scrollTop = logArea.scrollHeight;
    while (logArea.children.length > 500) logArea.removeChild(logArea.firstChild);
}
function escapeHtml(s) {
    return String(s).replace(/[&<>"']/g, c => ({
        '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
    })[c]);
}
$('#btnClearLog').onclick = () => { logArea.innerHTML = ''; };

// ---------- toast ----------
function toast(level, msg, ms = 2400) {
    const wrap = $('#toastWrap');
    const el = document.createElement('div');
    el.className = `toast t-${level}`;
    el.textContent = msg;
    wrap.appendChild(el);
    setTimeout(() => {
        el.style.transition = 'opacity .25s';
        el.style.opacity = '0';
        setTimeout(() => el.remove(), 280);
    }, ms);
}

// ============================================================================
//  全局状态
// ============================================================================
const state = {
    connected: false,
    motorIds: [],     // [int]
    limitsTurns: {},  // mid(str) -> {lo_turns, hi_turns}
    cfgUnit: 'turns',   // 服务端 cfg.pos_unit 的字符串
    statesPollTimer: null,
    statusPollTimer: null,
};

// ============================================================================
//  连接 / 断开 / 串口列表
// ============================================================================
async function refreshPorts() {
    const sel = $('#portSelect');
    const old = sel.value;
    const data = await api('/api/ports');
    sel.innerHTML = '';
    if (!data.ok || !data.ports) {
        sel.innerHTML = '<option value="">(无)</option>';
        return;
    }
    const cands = data.ports.filter(p => p.is_candidate);
    const others = data.ports.filter(p => !p.is_candidate);
    const buildOpt = (p, prefix = '') => {
        const o = document.createElement('option');
        o.value = p.port;
        o.textContent = `${prefix}${p.port}  ${p.vid ? `[${p.vid}:${p.pid}]` : ''}  ${p.description || ''}`;
        return o;
    };
    if (cands.length) {
        const g = document.createElement('optgroup'); g.label = '调试板候选';
        cands.forEach(p => g.appendChild(buildOpt(p, '★ ')));
        sel.appendChild(g);
    }
    if (others.length) {
        const g = document.createElement('optgroup'); g.label = '其它串口';
        others.forEach(p => g.appendChild(buildOpt(p)));
        sel.appendChild(g);
    }
    // 优先恢复上次选择, 否则选第一个候选
    if (old && Array.from(sel.options).some(o => o.value === old)) {
        sel.value = old;
    } else if (cands.length) {
        sel.value = cands[0].port;
    }
    $('#portHint').textContent =
        `共 ${data.ports.length} 个串口 (${cands.length} 个候选)`;
}

async function doConnect() {
    const port = $('#portSelect').value;
    if (!port) { toast('warn', '请先选择串口'); return; }
    const baud = parseInt($('#baudInput').value || '4000000', 10);
    const cfg  = $('#cfgPath').value.trim() || null;
    log('info', `连接 ${port} @ ${baud} bps...`);
    const data = await api('/api/connect', {
        method: 'POST',
        body: { port, baudrate: baud, cfg_path: cfg }
    });
    if (data.ok) {
        toast('ok', '已连接');
        log('ok', `已连接 ${port}: ${data.message || ''}`);
    } else {
        toast('error', data.message || '连接失败');
    }
    await refreshStatus();
}

async function doDisconnect() {
    if (!confirm('确认断开? 会先给所有电机发 stop.')) return;
    const data = await api('/api/disconnect', { method: 'POST' });
    if (data.ok) {
        toast('ok', '已断开');
        log('info', '已断开');
    }
    await refreshStatus();
}

// ============================================================================
//  状态拉取
// ============================================================================
async function refreshStatus() {
    const data = await api('/api/status');
    state.connected = !!data.connected;
    state.motorIds  = data.motor_ids || [];
    state.cfgUnit   = data.pos_unit  || 'turns';

    state.limitsTurns = {};
    if (data.limits) {
        Object.entries(data.limits).forEach(([mid, lim]) => {
            state.limitsTurns[mid] = {
                lo: lim.lo_turns, hi: lim.hi_turns,
                enabled: lim.enabled !== false,
            };
        });
    }

    // UI 同步
    $('#connBadge').textContent = state.connected
        ? `已连接 ${data.port || ''}`
        : '未连接';
    $('#connBadge').className = 'badge ' + (state.connected ? 'badge-on' : 'badge-off');

    $('#btnConnect').disabled    =  state.connected;
    $('#btnDisconnect').disabled = !state.connected;
    $('#btnHomeAll').disabled    = !state.connected;
    $('#btnStopAll').disabled    = !state.connected;
    $('#btnCanStat').disabled    = !state.connected;
    $('#btnSaveLimits').disabled = !state.connected;

    if (data.stats) {
        $('#statTick').textContent     = `tx ${data.stats.tx_frames}`;
        $('#statJitter').textContent   = `jit ${(data.stats.max_tx_jitter_ms || 0).toFixed(1)}ms`;
        $('#statRxAge').textContent    = `rx ${(data.stats.last_rx_age_ms || 0).toFixed(0)}ms`;
    } else {
        $('#statTick').textContent = '';
        $('#statJitter').textContent = '';
        $('#statRxAge').textContent = '';
    }

    // 重新建电机网格 (id 列表变化时)
    rebuildMotorGrid();

    // 同步全局 unit 下拉
    if (!state.userTouchedUnit) {
        $('#globalUnit').value = state.cfgUnit;
    }
}

// ============================================================================
//  电机网格
// ============================================================================
function rebuildMotorGrid() {
    const grid = $('#motorGrid');
    if (!state.connected || state.motorIds.length === 0) {
        grid.innerHTML = '<div class="empty-hint">尚未连接调试板; 请先在上方选择串口并点击 "连接"。</div>';
        return;
    }

    // 已存在的 id, 不重新创建
    const existing = new Set($$('.motor-card', grid).map(c => c.dataset.mid));
    const need     = new Set(state.motorIds.map(String));

    // 删除多余
    $$('.motor-card', grid).forEach(c => {
        if (!need.has(c.dataset.mid)) c.remove();
    });
    // 第一次清空 empty-hint
    const eh = grid.querySelector('.empty-hint');
    if (eh) eh.remove();

    // 新增缺失
    const tpl = $('#motorCardTpl').content;
    state.motorIds.forEach(mid => {
        if (existing.has(String(mid))) return;
        const node = tpl.firstElementChild.cloneNode(true);
        node.dataset.mid = String(mid);
        node.querySelector('.motor-id').textContent = `M${mid}`;
        grid.appendChild(node);
        wireMotorCard(node, mid);
    });

    // 应用一次限位 -> slider range
    state.motorIds.forEach(mid => {
        const card = grid.querySelector(`.motor-card[data-mid="${mid}"]`);
        if (card) applyLimitToCard(card, mid);
    });
}

function applyLimitToCard(card, mid) {
    const lim   = state.limitsTurns[String(mid)];
    const unit  = $('#globalUnit').value;
    const slider = card.querySelector('.pos-slider');
    const input  = card.querySelector('.pos-input');
    const limTxt = card.querySelector('.limit-text');
    const unitLabel = card.querySelector('.pos-unit-label');
    const loIn  = card.querySelector('.lim-lo-input');
    const hiIn  = card.querySelector('.lim-hi-input');
    unitLabel.textContent = unit;

    if (lim && lim.enabled !== false) {
        const lo = unitFromTurns(lim.lo, unit);
        const hi = unitFromTurns(lim.hi, unit);
        slider.min = lo.toFixed(4);
        slider.max = hi.toFixed(4);
        slider.step = ((hi - lo) / 1000).toFixed(5) || '0.001';
        // 把 input clamp 进范围
        const v = clamp(parseFloat(input.value) || 0, lo, hi);
        input.value = v.toFixed(4);
        slider.value = String(v);
        limTxt.textContent = `limit: [${lo.toFixed(3)}, ${hi.toFixed(3)}] ${unit}`;
        // 限位面板里的输入框: 只在用户没在编辑时同步, 避免抢覆盖
        if (loIn && document.activeElement !== loIn) loIn.value = lo.toFixed(4);
        if (hiIn && document.activeElement !== hiIn) hiIn.value = hi.toFixed(4);
    } else {
        slider.min = '-1'; slider.max = '1'; slider.step = '0.001';
        limTxt.textContent = 'limit: (未设)';
        if (loIn && document.activeElement !== loIn) loIn.value = '';
        if (hiIn && document.activeElement !== hiIn) hiIn.value = '';
    }
}

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

// 圈 -> 任意单位
function unitFromTurns(turns, unit) {
    if (unit === 'radians') return turns * 2 * Math.PI;
    if (unit === 'degrees') return turns * 360;
    return turns;
}
// 任意单位 -> 圈
function unitToTurns(value, unit) {
    if (unit === 'radians') return value / (2 * Math.PI);
    if (unit === 'degrees') return value / 360;
    return value;
}

function wireMotorCard(card, mid) {
    const slider = card.querySelector('.pos-slider');
    const input  = card.querySelector('.pos-input');
    slider.addEventListener('input', () => { input.value = parseFloat(slider.value).toFixed(4); });
    input.addEventListener('input',  () => {
        const v = parseFloat(input.value);
        if (!isNaN(v)) slider.value = String(v);
    });

    card.querySelector('.act-zero').onclick = () => {
        slider.value = '0'; input.value = '0';
    };
    card.querySelector('.act-move').onclick    = () => moveMotor(mid, parseFloat(input.value), card);
    card.querySelector('.act-stop').onclick    = () => stopMotor(mid);
    card.querySelector('.act-brake').onclick   = () => brakeMotor(mid);
    card.querySelector('.act-version').onclick = () => versionMotor(mid);
    card.querySelector('.act-resetzero').onclick = () => resetZeroMotor(mid);

    // Enter on input → move
    input.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') moveMotor(mid, parseFloat(input.value), card);
    });

    // ---- 限位标定面板 ----
    const limTog = card.querySelector('.act-limit-toggle');
    const limPan = card.querySelector('.limit-cal');
    limTog.onclick = () => {
        const open = limPan.hasAttribute('hidden');
        if (open) { limPan.removeAttribute('hidden'); limTog.classList.add('expanded'); limTog.textContent = '▴ 限位'; }
        else      { limPan.setAttribute('hidden',''); limTog.classList.remove('expanded'); limTog.textContent = '▾ 限位'; }
    };
    card.querySelector('.act-set-lo-cur').onclick = () => setCurrentAsLimit(mid, 'lo', card);
    card.querySelector('.act-set-hi-cur').onclick = () => setCurrentAsLimit(mid, 'hi', card);
    card.querySelector('.act-apply-lim').onclick  = () => applyLimit(mid, card);
    card.querySelector('.act-disable-lim').onclick = () => disableLimit(mid, card);
}

// ============================================================================
//  电机动作
// ============================================================================
async function moveMotor(mid, pos, card) {
    if (isNaN(pos)) { toast('warn', '位置无效'); return; }
    const unit = $('#globalUnit').value;
    const vel  = parseFloat($('#velMax').value) || 0.05;
    const acc  = parseFloat($('#accMax').value) || 0.05;
    setBusy(card, true);
    log('info', `M${mid} → ${pos} ${unit} (vel=${vel}, acc=${acc})`);
    const data = await api(`/api/motor/${mid}/move`, {
        method: 'POST',
        body: { pos, vel_max_rps: vel, acc_rpss: acc, unit }
    });
    setBusy(card, false);
    if (!data.ok) toast('error', `M${mid} 移动失败: ${data.message || ''}`);
}

async function stopMotor(mid) {
    log('warn', `M${mid} stop`);
    const data = await api(`/api/motor/${mid}/stop`, { method: 'POST' });
    if (data.ok) toast('ok', `M${mid} 已停止`);
}

async function brakeMotor(mid) {
    log('warn', `M${mid} brake`);
    const data = await api(`/api/motor/${mid}/brake`, { method: 'POST' });
    if (data.ok) toast('ok', `M${mid} 已刹车`);
}

async function versionMotor(mid) {
    const data = await api(`/api/motor/${mid}/version`);
    if (data.ok)  log('ok',  `M${mid} version: ${data.version}`);
    else          log('warn', `M${mid} version: 无回复`);
    toast(data.ok ? 'ok' : 'warn', `M${mid} ver: ${data.version || '无'}`);
}

async function resetZeroMotor(mid) {
    if (!confirm(
        `[危险] 把 M${mid} 当前位置写为零点?\n\n` +
        `这会改写电机内部零点配置. 通常需要把电机手动转到机械零位后再做.`
    )) return;
    log('warn', `M${mid} reset_zero (confirmed)`);
    const data = await api(`/api/motor/${mid}/reset_zero`, {
        method: 'POST', body: { confirm: true }
    });
    if (data.ok) toast('ok', `M${mid} set_zero 已发送`);
    else         toast('error', `M${mid} set_zero 失败: ${data.message || ''}`);
}

function setBusy(card, busy) {
    card.querySelectorAll('.btn').forEach(b => b.disabled = busy);
}

// ============================================================================
//  限位标定
// ============================================================================
function showLimitHint(card, msg, level = 'info') {
    const h = card.querySelector('.lc-hint');
    if (!h) return;
    h.textContent = msg;
    h.style.color = (level === 'ok')   ? 'var(--good)'
                  : (level === 'warn') ? 'var(--warn)'
                  : (level === 'err')  ? 'var(--bad)' : 'var(--text-2)';
    setTimeout(() => { if (h.textContent === msg) h.textContent = ''; }, 4000);
}

async function setCurrentAsLimit(mid, which, card) {
    log('info', `M${mid} 当前位置 → ${which}`);
    setBusy(card, true);
    const data = await api(`/api/motor/${mid}/limit/set_current_as`, {
        method: 'POST', body: { which }
    });
    setBusy(card, false);
    if (data.ok) {
        toast('ok', `M${mid} ${which} 已设为当前位置`);
        showLimitHint(card, `${which}=${data[which]?.toFixed?.(4)} ${data.unit || ''}`, 'ok');
        await refreshStatus();
    } else {
        toast('error', `M${mid} 标定失败: ${data.message || ''}`);
        showLimitHint(card, data.message || '失败', 'err');
    }
}

async function applyLimit(mid, card) {
    const lo = parseFloat(card.querySelector('.lim-lo-input').value);
    const hi = parseFloat(card.querySelector('.lim-hi-input').value);
    if (isNaN(lo) || isNaN(hi)) {
        toast('warn', 'lo / hi 必须是数字');
        showLimitHint(card, 'lo / hi 必须是数字', 'warn');
        return;
    }
    if (lo >= hi) {
        toast('warn', `lo(${lo}) >= hi(${hi}) 非法`);
        showLimitHint(card, `lo >= hi 非法`, 'warn');
        return;
    }
    const unit = $('#globalUnit').value;
    log('info', `M${mid} 设限位 [${lo}, ${hi}] ${unit}`);
    setBusy(card, true);
    const data = await api(`/api/motor/${mid}/limit`, {
        method: 'POST', body: { lo, hi, unit }
    });
    setBusy(card, false);
    if (data.ok) {
        toast('ok', `M${mid} 限位已更新`);
        showLimitHint(card, `已写: [${lo}, ${hi}]`, 'ok');
        await refreshStatus();
    } else {
        toast('error', `M${mid} 设限位失败: ${data.message || ''}`);
        showLimitHint(card, data.message || '失败', 'err');
    }
}

async function disableLimit(mid, card) {
    if (!confirm(`确认禁用 M${mid} 的软限位?\n\n禁用后驱动不会再 clamp 目标位置, 自己请小心.`)) return;
    log('warn', `M${mid} disable_position_limit`);
    const data = await api(`/api/motor/${mid}/limit/disable`, { method: 'POST' });
    if (data.ok) {
        toast('ok', `M${mid} 软限位已禁用`);
        showLimitHint(card, '已禁用', 'warn');
        await refreshStatus();
    } else {
        toast('error', data.message || '失败');
    }
}

// ============================================================================
//  全局动作
// ============================================================================
$('#btnHomeAll').onclick = async () => {
    if (!confirm('全部回零? 所有电机会向 0 位置移动.')) return;
    const vel = parseFloat($('#velMax').value) || 0.05;
    log('info', `home all (vel=${vel})`);
    const data = await api('/api/home_all', { method: 'POST', body: { vel_max_rps: vel } });
    toast(data.ok ? 'ok' : 'error', data.message || '');
};

$('#btnStopAll').onclick = async () => {
    log('warn', 'stop all');
    const data = await api('/api/stop_all', { method: 'POST' });
    toast(data.ok ? 'ok' : 'error', data.message || '');
};

$('#btnCanStat').onclick = async () => {
    const d = await api('/api/can_status');
    log(d.ok ? 'ok' : 'warn',
        `CAN status: fault=${d.fault} lec=${d.lec} txErr=${d.tx_err_count} rxErr=${d.rx_err_count}\n  raw=${d.raw}`);
    toast(d.ok ? 'ok' : 'warn', `CAN ${d.fault}`);
};

$('#btnSaveLimits').onclick = async () => {
    if (!confirm('把当前所有运行时限位写回 robot.cfg?\n\n原文件会先备份成 robot.cfg.bak.')) return;
    log('info', 'save limits to cfg...');
    const data = await api('/api/limits/save', { method: 'POST', body: {} });
    if (data.ok) {
        toast('ok', `已保存 ${data.saved_limits} 条限位 (单位 ${data.unit})`);
        log('ok', `已保存到 ${data.path} (备份: ${data.backup || '无'})`);
    } else {
        toast('error', data.message || '保存失败');
    }
};

$('#globalUnit').addEventListener('change', () => {
    state.userTouchedUnit = true;
    state.motorIds.forEach(mid => {
        const card = $(`.motor-card[data-mid="${mid}"]`);
        if (card) applyLimitToCard(card, mid);
    });
});

$('#btnRefreshPorts').onclick = refreshPorts;
$('#btnConnect').onclick    = doConnect;
$('#btnDisconnect').onclick = doDisconnect;

// ============================================================================
//  100 ms 状态轮询 → 刷新所有 card
// ============================================================================
async function pollStates() {
    if (!state.connected) return;
    const data = await api('/api/states');
    if (!data.ok) return;
    const unit = $('#globalUnit').value;

    Object.entries(data.states || {}).forEach(([mid, s]) => {
        const card = $(`.motor-card[data-mid="${mid}"]`);
        if (!card) return;
        if (!s) {
            card.classList.add('disconnected');
            card.querySelector('.pos').textContent = '—';
            card.querySelector('.vel').textContent = '—';
            card.querySelector('.trq').textContent = '—';
            return;
        }
        card.classList.remove('disconnected');

        // pos: 后端返回的是 cfg unit 的值 + position_turns; 这里按 UI 当前 unit 重算
        const posUI = unitFromTurns(s.position_turns, unit);
        card.querySelector('.pos').textContent = fmt(posUI, 4);
        card.querySelector('.vel').textContent = fmt(s.velocity, 4);
        card.querySelector('.trq').textContent = fmt(s.torque, 0);

        const modeBadge = card.querySelector('.motor-mode-badge');
        modeBadge.textContent = `mode ${s.mode}`;
        modeBadge.className = `motor-mode-badge mode-${s.mode}`;

        const fb = card.querySelector('.motor-fault-badge');
        if (s.fault && s.fault !== 0) {
            fb.textContent = `fault ${s.fault}`;
            fb.className = 'motor-fault-badge fault';
            card.classList.add('fault');
        } else {
            fb.textContent = '';
            fb.className = 'motor-fault-badge empty';
            card.classList.remove('fault');
        }

        const lb = card.querySelector('.motor-limit-badge');
        if (s.pos_limit_flag === 1) {
            lb.textContent = '↑ over limit';
            lb.className = 'motor-limit-badge up';
            card.classList.add('limit-warn');
        } else if (s.pos_limit_flag === -1) {
            lb.textContent = '↓ under limit';
            lb.className = 'motor-limit-badge down';
            card.classList.add('limit-warn');
        } else {
            lb.textContent = '';
            lb.className = 'motor-limit-badge empty';
            card.classList.remove('limit-warn');
        }
    });
}

// ============================================================================
//  启动
// ============================================================================
(async function init() {
    await refreshPorts();
    await refreshStatus();
    state.statesPollTimer = setInterval(pollStates, 120);
    state.statusPollTimer = setInterval(refreshStatus, 1500);
    log('info', '前端已启动. 选择串口后点 "连接".');
})();
