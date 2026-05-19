import math
import random
import socket
import sys
import threading
import time

try:
    import pygame
except ImportError:
    print("pygame not found.  Install with:  pip install pygame")
    sys.exit(1)

# ═══════════════════════════════════════════════════════════════════════════════
# Physical parameters  — edit freely
# ═══════════════════════════════════════════════════════════════════════════════
M   = 1.0     # cart mass  (kg)
m   = 0.1     # pole mass  (kg)
l   = 0.5     # pole half-length (m)  — pivot to COM
g   = 9.81    # gravity (m/s²)
dt  = 0.005   # simulation time-step (s)  — 5 ms

X_LIMIT = 2.5  # cart position hard limits ±X_LIMIT (m)

# ═══════════════════════════════════════════════════════════════════════════════
# Shared simulation state
# ═══════════════════════════════════════════════════════════════════════════════
# theta is CONTINUOUS (no wrapping). 0 = down, π = up, can be any real number.
sim = {
    "x":         0.0,
    "theta":     0.0,          # start upright (π)
    "x_dot":     0.0,
    "theta_dot": 0.0,
}
cmd_vel    = 0.0          # commanded cart velocity (m/s)
tcp_connected = False

state_lock = threading.Lock()   # protects sim dict
cmd_lock   = threading.Lock()   # protects cmd_vel


# ═══════════════════════════════════════════════════════════════════════════════
# Physics
# ═══════════════════════════════════════════════════════════════════════════════

def wrap_2pi(angle):
    """Wrap angle to [0, 2π) for display only."""
    return angle % (2 * math.pi)


def step_physics(x, theta, x_dot, theta_dot, v_cmd):
    """
    Returns new continuous (x, theta, x_dot, theta_dot).
    Soft walls replace the old hard limits.
    """
    s = math.sin(theta)
    c = math.cos(theta)

    # damping
    cart_damping = 2.5
    pole_damping = 0.4

    # velocity servo -> force conversion
    Kv = 40.0
    F = Kv * (v_cmd - x_dot)
    F = max(min(F, 30.0), -30.0)   # actuator saturation

    # ========== Soft wall force ==========
    # Prevents the cart from leaving the track without abrupt collisions
    wall_stiffness = 500.0   # N/m  (very stiff)
    wall_damping   = 50.0    # N/(m/s)
    margin = 0.05            # start acting 5 cm before the limit

    F_wall = 0.0
    if x > X_LIMIT - margin:
        # penetration beyond the soft zone
        delta = x - (X_LIMIT - margin)
        F_wall = -wall_stiffness * delta - wall_damping * x_dot
    elif x < -X_LIMIT + margin:
        delta = x + (X_LIMIT - margin)
        F_wall = -wall_stiffness * delta - wall_damping * x_dot

    F += F_wall
    # ====================================

    # nonlinear cart-pole dynamics (theta=0 down, theta=π up)
    denom = M + m * s * s

    x_ddot = (
        F
        + m * s * (l * theta_dot**2 + g * c)
        - cart_damping * x_dot
    ) / denom

    theta_ddot = (
        -F * c
        - m * l * theta_dot**2 * c * s
        - (M + m) * g * s
        - pole_damping * theta_dot
    ) / (l * denom)

    # integrate
    x_dot += x_ddot * dt
    x += x_dot * dt

    theta_dot += theta_ddot * dt
    theta += theta_dot * dt

    # process noise
    theta += random.gauss(0.0, 0.00015)
    theta_dot += random.gauss(0.0, 0.0005)
    x += random.gauss(0.0, 0.00002)
    x_dot += random.gauss(0.0, 0.0001)

    # (No hard clamp — soft walls already keep x inside ±X_LIMIT)
    return x, theta, x_dot, theta_dot


# ═══════════════════════════════════════════════════════════════════════════════
# Simulation thread  (200 Hz wall-clock)
# ═══════════════════════════════════════════════════════════════════════════════

def simulation_loop():
    print(f"[SIM] dt={dt*1000:.0f} ms  M={M}kg  m={m}kg  l={l}m  limits=±{X_LIMIT}m")
    next_tick = time.perf_counter()
    while True:
        next_tick += dt
        sleep_for = next_tick - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)

        with cmd_lock:
            u = cmd_vel

        with state_lock:
            x, th, xd, td = (sim["x"], sim["theta"],
                              sim["x_dot"], sim["theta_dot"])

        x, th, xd, td = step_physics(x, th, xd, td, u)

        with state_lock:
            sim["x"]         = x
            sim["theta"]     = th
            sim["x_dot"]     = xd
            sim["theta_dot"] = td


# ═══════════════════════════════════════════════════════════════════════════════
# TCP server  (one thread per client)
# ═══════════════════════════════════════════════════════════════════════════════

def handle_client(conn, addr):
    """
    Protocol:
      Receive: plain float string ending with \\n   e.g.  "0.35\\n"
      Send:    "x=<f>,theta=<f>,xdot=<f>,thetadot=<f>\\n"
      theta is the CONTINUOUS (unwrapped) angle, not wrapped to [0,2π).
    """
    global cmd_vel, tcp_connected
    tcp_connected = True
    print(f"[TCP] Client connected: {addr}")
    buf = b""
    try:
        while True:
            chunk = conn.recv(256)
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode("utf-8", errors="ignore").strip()
                if text:
                    try:
                        v = float(text)
                        with cmd_lock:
                            cmd_vel = v
                            print(f"[TCP] u: {v:.4f} m/s")
                    except ValueError:
                        pass

                # Reply with current state (continuous theta)
                with state_lock:
                    sx  = sim["x"]
                    sth_cont = sim["theta"]   # continuous, no wrap
                    sxd = sim["x_dot"]
                    std = sim["theta_dot"]

                reply = (f"{sx:.6f},{sth_cont:.6f},{sxd:.6f},{std:.6f}\n")
                try:
                    conn.sendall(reply.encode())
                except OSError:
                    return
    except (ConnectionResetError, OSError):
        pass
    finally:
        conn.close()
        tcp_connected = False
        with cmd_lock:
            cmd_vel = 0.0
        print(f"[TCP] Client disconnected: {addr}")


def tcp_server_loop():
    host, port = "0.0.0.0", 8080
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen(5)
        print(f"[TCP] Listening on {host}:{port}")
        while True:
            conn, addr = srv.accept()
            threading.Thread(target=handle_client,
                             args=(conn, addr), daemon=True).start()


# ═══════════════════════════════════════════════════════════════════════════════
# Visualizer
# ═══════════════════════════════════════════════════════════════════════════════

# --- display constants --------------------------------------------------------
W, H       = 1100, 680
FPS        = 60
TRACK_Y    = 430          # screen-y of the rail
SCALE      = 210          # pixels per metre
CX         = W // 2      # screen-x when cart x=0

# colours
C_BG       = (235, 235, 232)
C_RAIL     = (45,  45,  45)
C_GRID     = (210, 210, 207)
C_LIMIT    = (180,  40,  40)
C_CART     = (52,  118, 210)
C_CART_SH  = (28,  76, 158)
C_POLE     = (210,  38,  38)
C_PIVOT    = (20,   20,  20)
C_HUD_BG   = (255, 255, 255)
C_HUD_BD   = (195, 195, 192)
C_TITLE    = (30,   30,  30)
C_KEY      = (100, 100, 100)
C_OK       = (25,  130,  55)
C_WARN     = (195,  90,  10)
C_BAD      = (185,  28,  28)
C_BAR_BG   = (215, 215, 212)
C_TICK     = (145, 145, 140)
C_WM       = (185, 185, 180)


def w2s(wx, wy):
    """World (m) → screen (px).  World +y = up."""
    return int(CX + wx * SCALE), int(TRACK_Y - wy * SCALE)


def vcol(v, warn, bad):
    a = abs(v)
    return C_BAD if a >= bad else (C_WARN if a >= warn else C_OK)


def rr(surf, color, rect, rad=8, bw=0, bc=None):
    pygame.draw.rect(surf, color, rect, border_radius=rad)
    if bw and bc:
        pygame.draw.rect(surf, bc, rect, bw, border_radius=rad)


def draw_bar(surf, x, y, w, h, val, lo, hi, col):
    pygame.draw.rect(surf, C_BAR_BG, (x, y, w, h), border_radius=3)
    frac = max(0.0, min(1.0, (val - lo) / (hi - lo)))
    fw = int(w * frac)
    if fw > 0:
        pygame.draw.rect(surf, col, (x, y, fw, h), border_radius=3)


def draw_scene(surf, fonts, sx_cart, theta_cont, l_m):
    """Draw rail, limit stops, cart, pole, pivot.
       theta_cont is continuous; we wrap it for visual consistency."""
    theta = wrap_2pi(theta_cont)   # wrap for drawing

    # grid ticks
    for i in range(-10, 11):
        xw = i * 0.1
        sx, _ = w2s(xw, 0)
        pygame.draw.line(surf, C_GRID, (sx, TRACK_Y - 55), (sx, TRACK_Y + 16), 1)

    # rail
    lx, _ = w2s(-X_LIMIT, 0)
    rx, _ = w2s( X_LIMIT, 0)
    pygame.draw.line(surf, C_RAIL, (lx, TRACK_Y), (rx, TRACK_Y), 5)

    # limit stops
    for lim in (-X_LIMIT, X_LIMIT):
        sx, _ = w2s(lim, 0)
        pygame.draw.line(surf, C_LIMIT, (sx, TRACK_Y - 45), (sx, TRACK_Y + 42), 4)

    # origin tick
    ox, _ = w2s(0, 0)
    pygame.draw.line(surf, C_TICK, (ox, TRACK_Y - 12), (ox, TRACK_Y + 12), 2)

    # x-axis labels
    for i in range(-5, 6):
        xw = i * 0.2
        sx, _ = w2s(xw, 0)
        lbl = fonts["tiny"].render(f"{xw:.1f}", True, C_TICK)
        surf.blit(lbl, (sx - lbl.get_width() // 2, TRACK_Y + 18))

    # ── cart ──────────────────────────────────────────────────────────────────
    cw, ch = 104, 46
    cart_rect = pygame.Rect(sx_cart - cw // 2, TRACK_Y - ch, cw, ch)

    # shadow
    shadow_surf = pygame.Surface((cw, ch), pygame.SRCALPHA)
    pygame.draw.rect(shadow_surf, (0, 0, 0, 45), (0, 0, cw, ch), border_radius=8)
    surf.blit(shadow_surf, (cart_rect.x + 4, cart_rect.y + 5))

    rr(surf, C_CART, cart_rect, rad=8)
    # shine strip
    pygame.draw.rect(surf, (95, 158, 240),
                     pygame.Rect(cart_rect.x + 7, cart_rect.y + 6, cw - 14, 10),
                     border_radius=4)
    # bottom accent
    pygame.draw.rect(surf, C_CART_SH,
                     pygame.Rect(cart_rect.x, cart_rect.bottom - 11, cw, 11),
                     border_radius=8)
    # wheels
    for wx in (sx_cart - 30, sx_cart + 30):
        pygame.draw.circle(surf, (55, 55, 55), (wx, TRACK_Y + 7), 10)
        pygame.draw.circle(surf, (120, 120, 120), (wx, TRACK_Y + 7), 5)

    # ── pole ──────────────────────────────────────────────────────────────────
    # theta = 0 → DOWN, theta = π → UP
    pivot_sy = TRACK_Y - ch // 2
    l_px = l_m * SCALE * 2          # full visual length (2×half-length)

    tip_sx = sx_cart + int(l_px * math.sin(theta))
    tip_sy = pivot_sy + int(l_px * math.cos(theta))

    pygame.draw.line(surf, C_POLE, (sx_cart, pivot_sy), (tip_sx, tip_sy), 8)
    pygame.draw.circle(surf, C_POLE,  (tip_sx,  tip_sy),  7)
    pygame.draw.circle(surf, C_PIVOT, (sx_cart, pivot_sy), 9)
    pygame.draw.circle(surf, (80, 80, 80), (sx_cart, pivot_sy), 5)


def draw_hud(surf, fonts, s_cont, u, conn):
    """s_cont contains continuous theta; we wrap it for display."""
    hx, hy, hw, hh = 26, 26, 295, 390
    rr(surf, C_HUD_BG, (hx, hy, hw, hh), rad=12, bw=1, bc=C_HUD_BD)

    # title
    surf.blit(fonts["title"].render("Cart Pole", True, C_TITLE), (hx + 16, hy + 14))

    # connection badge
    dot_c = (35, 175, 75) if conn else (195, 45, 45)
    pygame.draw.circle(surf, dot_c, (hx + hw - 20, hy + 22), 7)
    surf.blit(fonts["small"].render("LIVE" if conn else "OFFLINE", True, dot_c),
              (hx + hw - 66, hy + 16))

    pygame.draw.line(surf, C_HUD_BD, (hx + 12, hy + 46), (hx + hw - 12, hy + 46), 1)

    theta_wrapped = wrap_2pi(s_cont["theta"])

    rows = [
        ("x",         f"{s_cont['x']:+.4f} m",         vcol(s_cont['x'],        0.75, 0.95),
         s_cont['x'],        -X_LIMIT, X_LIMIT),
        ("x_dot",     f"{s_cont['x_dot']:+.4f} m/s",   vcol(s_cont['x_dot'],    1.5,  3.0),
         s_cont['x_dot'],    -3.0,    3.0),
        ("theta",     f"{theta_wrapped:+.4f} rad",      vcol(theta_wrapped,    2.0,  2.9),
         theta_wrapped,      0.0,     2*math.pi),
        ("theta_dot", f"{s_cont['theta_dot']:+.4f} r/s",vcol(s_cont['theta_dot'],3.0,  6.0),
         s_cont['theta_dot'],-8.0,    8.0),
        ("u",         f"{u:+.4f} m/s",                 C_OK,
         u,             -2.0,    2.0),
    ]

    ry = hy + 58
    for label, valstr, col, raw, lo, hi in rows:
        surf.blit(fonts["mono_s"].render(label, True, C_KEY),  (hx + 16, ry))
        surf.blit(fonts["mono"].render(valstr, True, col),      (hx + 16, ry + 17))
        draw_bar(surf, hx + 16, ry + 38, hw - 32, 6, raw, lo, hi, col)
        ry += 64

    pygame.draw.line(surf, C_HUD_BD,
                     (hx + 12, hy + hh - 50), (hx + hw - 12, hy + hh - 50), 1)
    surf.blit(fonts["tiny"].render("← → : velocity  |  SPACE : stop  |  ESC : quit",
                                   True, C_TICK), (hx + 10, hy + hh - 36))


def run_visualizer():
    global cmd_vel

    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Cart-Pole  —  Live Simulation")
    clock = pygame.time.Clock()

    fonts = {
        "title":  pygame.font.SysFont("Georgia",     20, bold=True),
        "mono":   pygame.font.SysFont("Courier New", 15, bold=True),
        "mono_s": pygame.font.SysFont("Courier New", 11),
        "small":  pygame.font.SysFont("Arial",       11, bold=True),
        "tiny":   pygame.font.SysFont("Arial",       10),
    }

    L_DRAW = l   # pole half-length for rendering

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    pygame.quit(); sys.exit()
                elif event.key == pygame.K_RIGHT:
                    with cmd_lock: cmd_vel =  0.5
                elif event.key == pygame.K_LEFT:
                    with cmd_lock: cmd_vel = -0.5
                elif event.key == pygame.K_SPACE:
                    with cmd_lock: cmd_vel =  0.0
            elif event.type == pygame.KEYUP:
                if event.key in (pygame.K_LEFT, pygame.K_RIGHT):
                    with cmd_lock: cmd_vel = 0.0

        # snapshot state (continuous)
        with state_lock:
            s_cont = dict(sim)
        with cmd_lock:
            u = cmd_vel

        # world → screen for cart
        sx_cart, _ = w2s(s_cont["x"], 0)

        screen.fill(C_BG)
        draw_scene(screen, fonts, sx_cart, s_cont["theta"], L_DRAW)
        draw_hud(screen, fonts, s_cont, u, tcp_connected)

        # watermark
        wm = fonts["tiny"].render(
            "Cart-Pole Simulator  •  TCP 0.0.0.0:8080", True, C_WM)
        screen.blit(wm, (W - wm.get_width() - 14, H - 20))

        pygame.display.flip()
        clock.tick(FPS)


# ═══════════════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    # 1. physics loop
    threading.Thread(target=simulation_loop, daemon=True).start()
    # 2. TCP server
    threading.Thread(target=tcp_server_loop,  daemon=True).start()
    # 3. pygame on main thread (required on macOS / Windows)
    run_visualizer()
