# -*- coding: utf-8 -*-
"""
pid_sim.py — 在 PC 上逐行复现 My_PID_1_v 固件的 TIM3 中断控制逻辑
=================================================================
固件对应关系 (以 2026-10-25 09:14 版 tim.c 为准):
  TIM1  : 编码器接口 (TI12 4倍频), 16bit 计数器          -> pos_counts
  TIM2  : 10kHz PWM, CCR 0..100                          -> duty
  TIM3  : 100Hz 周期中断, 跑双环 PID                      -> tick()
  PB10/11: H桥方向
  post_pid(0.10,0.30,0.01): 速度环, target=v_target(现为100rpm)
  motor_pid (1.00,0.30,0.01): "电流环", actual=上一拍PWM(伪内环)

场景:
  old_buggy : 09:14 之前的固件 (无cnt_prev更新, 0.1缩放, target=2500)
  as_is     : 当前固件 (已补cnt_prev, 无缩放, target=100, 伪内环, 无AW)
  as_is_aw  : 当前固件 + 积分钳位抗饱和
  tuned     : 推荐方案: 去掉伪内环, 单环速度PID直接输出占空比 (IMC整定+AW)

电机模型(假设): 一阶惯性 omega -> duty*RPM_PER_PCT, 时间常数 TAU
编码器: 390线 x4倍频 = 1560 counts/圈 (由固件系数 3.846 = 100Hz*60/1560 反推)
t>=1.5s 施加负载扰动 (等效稳态转速被拉低 50rpm)
"""

CPR = 1560            # counts per rev (4x 编码器模式)
DT = 0.01             # 100Hz 控制周期
SUB = 20              # 物理子步
RPM_PER_PCT = 300.0   # 占空比1% -> 空载稳态300rpm (假设电机常数)
TAU = 0.030           # 机电时间常数 30ms
LOAD_RPM = 50.0       # 负载扰动等效转速降落
LOAD_T = 1.5          # 加载时刻 s


def wrap16(x):
    """C: int16 回绕"""
    x = int(x) & 0xFFFF
    return x - 0x10000 if x >= 0x8000 else x


def c_cast_i16(f):
    """Cortex-M3: float -> int16_t, VCVT 向零截断 + 饱和(不回绕)"""
    i = int(f)
    return max(-32768, min(32767, i))


class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.target = self.actual = 0.0
        self.error = self.last_error = 0.0
        self.integral = 0.0
        self.out = 0.0


def pid_calc(p, target, actual, anti_windup=False, MAX=100.0, MIN=-100.0):
    """逐行对应 MDK-ARM/pid.c 的 PID_Calc
    anti_windup=True 时加教科书式"条件积分/钳位法":
      输出已饱和且误差仍在推向饱和 -> 本拍不累积积分"""
    p.target = target
    p.actual = actual
    p.error = p.target - p.actual
    d_term = (p.error - p.last_error) * p.kd
    out = p.error * p.kp + p.ki * p.integral + d_term   # 先用旧积分试探饱和
    if anti_windup and ((out > MAX and p.error > 0) or (out < MIN and p.error < 0)):
        pass                                            # 冻结积分
    else:
        p.integral += p.error
        out = p.error * p.kp + p.ki * p.integral + d_term
    if out > MAX:
        out = MAX
    elif out < MIN:
        out = MIN
    p.last_error = p.error
    return out


SCENARIOS = {
    #              v_target scale  fix_cnt  aw     inner(伪内环)  外环增益                   内环增益
    "old_buggy":  dict(v_target=2500.0, scale=0.1, fix_cnt=False, aw=False, inner=True,  gains=(0.10, 0.30, 0.01)),
    "as_is_0914": dict(v_target=100.0,  scale=1.0, fix_cnt=True,  aw=False, inner=True,  gains=(0.10, 0.30, 0.01), inner_gains=(1.00, 0.30, 0.01)),
    "as_is_0919": dict(v_target=100.0,  scale=1.0, fix_cnt=True,  aw=False, inner=True,  gains=(0.10, 0.30, 0.01), inner_gains=(0.09, 0.90, 0.10)),
    "as_is_aw":   dict(v_target=100.0,  scale=1.0, fix_cnt=True,  aw=True,  inner=True,  gains=(0.10, 0.30, 0.01), inner_gains=(0.09, 0.90, 0.10)),
    "tuned":      dict(v_target=100.0,  scale=1.0, fix_cnt=True,  aw=True,  inner=False, gains=(0.008, 0.0025, 0.0)),
    # tuned + 高分辨率PWM: TIM2 Period=1000-1 (1kHz), CCR 0..1000, 占空比分辨率0.1%
    "tuned_hr":   dict(v_target=100.0,  scale=1.0, fix_cnt=True,  aw=True,  inner=False, gains=(0.008, 0.0025, 0.0), hr=True),
}


def simulate(cfg, t_end=2.0):
    post_pid = PID(*cfg["gains"])
    motor_pid = PID(*cfg.get("inner_gains", (1.00, 0.30, 0.01)))

    omega = 0.0        # 真实转速 rpm (带符号)
    pos = 0.0          # 累计编码器 counts (float, 带符号)
    cnt_prev = 0       # tim.c: int16_t cnt_prev
    motor_pwm = 0      # main.c: volatile int16_t motor_pwm (存绝对值!)

    log = []
    ticks = int(t_end / DT)
    for k in range(ticks):
        # ---- 固件 HAL_TIM_PeriodElapsedCallback 逐行复现 ----
        cnt_u16 = int(pos) % 65536          # __HAL_TIM_GET_COUNTER(&htim1)
        now = wrap16(cnt_u16)               # int16_t now
        cnt = now                           # volatile uint16_t cnt = now
        v_gap = wrap16(cnt - cnt_prev)      # int16_t v_gap = cnt - cnt_prev
        if cfg["fix_cnt"]:
            cnt_prev = cnt                  # <<< 09:14 补上的一行
        v_actuall = c_cast_i16(3.846 * v_gap)   # int16_t <- 3.846f*(float)v_gap

        if cfg["inner"]:
            motor_pid.target = cfg["scale"] * pid_calc(post_pid, cfg["v_target"], v_actuall, cfg["aw"])
            pwm_f = pid_calc(motor_pid, motor_pid.target, motor_pwm, cfg["aw"])
        else:
            # 推荐方案: 速度环直接输出占空比
            motor_pid.target = 0.0
            pwm_f = pid_calc(post_pid, cfg["v_target"], v_actuall, cfg["aw"])
        motor_pwm = c_cast_i16(pwm_f)

        if cfg.get("hr"):
            # 高分辨率PWM: CCR = round(out*10), 周期1000 -> 占空比按0.1%量化
            duty_pct = round(pwm_f * 10.0) / 10.0
            duty_pct = max(-100.0, min(100.0, duty_pct))
            duty_signed = duty_pct
            load = LOAD_RPM if k * DT >= LOAD_T else 0.0
            for _ in range(SUB):
                omega += (duty_signed * RPM_PER_PCT - load - omega) * (DT / SUB / TAU)
                pos += omega / 60.0 * CPR * (DT / SUB)
            log.append(dict(t=k * DT, v_act=v_actuall, omega=omega,
                            cnt=cnt, pos=pos % 65536, pwm=duty_signed,
                            target=motor_pid.target, integ=post_pid.integral))
            continue

        if motor_pwm < 0:                   # 反转: PB10=1 PB11=0
            direction = -1
            motor_pwm = -motor_pwm
        else:                               # 正转: PB10=0 PB11=1
            direction = +1
        if motor_pwm > 100:
            motor_pwm = 100
        duty_signed = direction * motor_pwm
        # __HAL_TIM_SET_COMPARE(&htim2, CH1, motor_pwm)

        # ---- 物理仿真 (电机+编码器), 每拍 SUB 个子步 ----
        load = LOAD_RPM if k * DT >= LOAD_T else 0.0
        for _ in range(SUB):
            omega += (duty_signed * RPM_PER_PCT - load - omega) * (DT / SUB / TAU)
            pos += omega / 60.0 * CPR * (DT / SUB)

        log.append(dict(t=k * DT, v_act=v_actuall, omega=omega,
                        cnt=cnt, pos=pos % 65536, pwm=duty_signed,
                        target=motor_pid.target, integ=post_pid.integral))
    return log


# ---------------------- 绘图 (无 matplotlib, 用 Pillow 手画) ----------------------
from PIL import Image, ImageDraw, ImageFont

W, H = 1500, 1060
img = Image.new("RGB", (W, H), "white")
d = ImageDraw.Draw(img)

def font(sz):
    for p in ("C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/simhei.ttf",
              "C:/Windows/Fonts/arial.ttf"):
        try:
            return ImageFont.truetype(p, sz)
        except OSError:
            continue
    return ImageFont.load_default()

F_T, F_L = font(26), font(17)

def panel(x0, y0, w, h, title, ymin, ymax):
    d.rectangle([x0, y0, x0 + w, y0 + h], outline="#333", width=2)
    d.text((x0 + 10, y0 + 6), title, font=F_T, fill="black")
    for i in range(6):
        yy = y0 + h - i * h / 5.0
        val = ymin + (ymax - ymin) * i / 5.0
        d.line([x0, yy, x0 + w, yy], fill="#e0e0e0")
        d.text((x0 - 78, yy - 9), f"{val:.0f}", font=F_L, fill="#555")
    for i in range(9):
        xx = x0 + i * w / 8.0
        d.line([xx, y0, xx, y0 + h], fill="#f0f0f0")
        d.text((xx - 18, y0 + h + 4), f"{i * 250}", font=F_L, fill="#555")
    d.text((x0 + w // 2 - 40, y0 + h + 4), "时间 (ms)", font=F_L, fill="#555")
    def map_pt(t, v):
        vx = min(max(v, ymin), ymax)
        return (x0 + t / 2000.0 * w, y0 + h - (vx - ymin) / (ymax - ymin) * h)
    return map_pt

def curve(map_pt, log, key, color, width=2, dash=0, scale=1.0):
    pts = [(map_pt(r["t"] * 1000, r[key] * scale)) for r in log]
    if dash:
        for i in range(len(pts) - 1):
            if i % (dash * 2) < dash:
                d.line([pts[i], pts[i + 1]], fill=color, width=width)
    else:
        d.line(pts, fill=color, width=width)

def load_line(p, y0, h):
    x = p(1500, 0)[0]
    for yy in range(int(y0), int(y0 + h), 12):
        d.line([x, yy, x, min(yy + 6, y0 + h)], fill="#c080e0", width=2)
    d.text((x + 6, y0 + 34), "1.5s 加载", font=F_L, fill="#9050c0")

logs = {m: simulate(c) for m, c in SCENARIOS.items()}

# ---- 面板1: 转速 ----
p1 = panel(110, 60, 1330, 380, "① 转速响应: 固件09:14版(红) / 09:19调参版(橙) / 推荐单环(绿) / 单环+高分辨率PWM(青), 目标100rpm", -1600, 4600)
load_line(p1, 60, 380)
t_ms = [r["t"] * 1000 for r in logs["as_is_0914"]]
pts = [p1(t, 100) for t in t_ms]
for i in range(len(pts) - 1):
    if i % 12 < 6:
        d.line([pts[i], pts[i + 1]], fill="#999", width=2)
d.text((1150, p1(0, 100)[1] + 6), "目标 100 rpm", font=F_L, fill="#666")
curve(p1, logs["as_is_0914"], "omega", "#e03030", 3)
curve(p1, logs["as_is_0919"], "omega", "#e09000", 2)
curve(p1, logs["tuned"], "omega", "#109030", 3)
curve(p1, logs["tuned_hr"], "omega", "#008080", 3)

# ---- 面板2: 编码器计数 ----
p2 = panel(110, 520, 1330, 380, "② TIM1 编码器计数 cnt (int16 视角): 伪内环版剧烈正反转/满占空锁存, 推荐方案平稳单向旋转", -33000, 33000)
load_line(p2, 520, 380)
curve(p2, logs["as_is_0914"], "cnt", "#e03030", 2)
curve(p2, logs["as_is_0919"], "cnt", "#e09000", 2)
curve(p2, logs["tuned"], "cnt", "#109030", 3)
curve(p2, logs["tuned_hr"], "cnt", "#008080", 2)

# ---- 图例 ----
lx, ly = 130, 935
items = [("#e03030", "as_is_0914: 内环(1.0,0.3,0.01) → 满占空锁存"),
         ("#e09000", "as_is_0919: 你最新调参 内环(0.09,0.9,0.1)"),
         ("#109030", "tuned: 单环速度PID kp=0.008 ki=0.0025 +AW"),
         ("#008080", "tuned_hr: tuned + 高分辨率PWM(Period=1000)"),
         ("#999999", "目标 100rpm(灰虚线)"),
         ("#c080e0", "1.5s 施加负载扰动(-50rpm等效)"),
         ("#000000", "注: 电机模型为假设参数(1%占空比≈300rpm空载,τ=30ms), 定性结论不依赖具体常数")]
for i, (c, s) in enumerate(items):
    x = lx + (i % 3) * 470
    y = ly + (i // 3) * 28
    d.line([x, y + 9, x + 34, y + 9], fill=c, width=4)
    d.text((x + 42, y), s, font=F_L, fill="#222")

img.save("REVIEW_BY_AGENT/sim_results.png")

# ---- 文本摘要 ----
def summarize(m):
    log = logs[m]
    tail = log[-30:]
    om = [r["omega"] for r in tail]
    va = [r["v_act"] for r in tail]
    du = [abs(r["pwm"]) for r in log]
    print(f"[{m:9s}] tail0.3s: omega min/max = {min(om):7.0f}/{max(om):7.0f} rpm, "
          f"v_act min/max = {min(va):7.0f}/{max(va):7.0f}, "
          f"max|duty| = {max(du):3.0f}%, cnt_end = {log[-1]['cnt']:6d}")

print("SIM DONE (motor model: 1%duty->300rpm, tau=30ms, 1560cpr)")
for m in logs:
    summarize(m)

# ---- 详细轨迹 (每100ms一行) 写入 UTF-8 文件, 避免控制台乱码 ----
with open("REVIEW_BY_AGENT/sim_trace.txt", "w", encoding="utf-8") as f:
    for m in logs:
        f.write(f"===== mode={m} =====\n")
        f.write("  t(ms)  omega_rpm  v_actuall  duty%  inner_tgt  post_integ\n")
        for i, r in enumerate(logs[m]):
            if i % 10 == 0:
                f.write(f"{r['t']*1000:7.0f} {r['omega']:10.1f} {r['v_act']:10d} "
                        f"{r['pwm']:7.1f} {r['target']:9.2f} {r['integ']:10.1f}\n")
print("trace -> REVIEW_BY_AGENT/sim_trace.txt")
