# environment_socket_write_then_read_float_forgiving.py
# SEND Tin as plain float line -> RECV setpoints (robust: \n or \0, text or 2*double) -> STEP
# Accepts: "[a,b]", "a,b", "{a,b}", "a b", and raw 16-byte binary doubles.

import gymnasium as gym
import numpy as np
import socket, json, os, csv, struct, math
import sinergym
from datetime import datetime
from gymnasium.wrappers import TimeLimit

# -------------------- SETTINGS --------------------
HOST, PORT = "0.0.0.0", 9000
SOCKET_TIMEOUT = 30.0
DEADBAND = 1.0
T_ROOM_KEY = "west_zone_air_temperature"
T_OUT_KEY  = "outdoor_temperature"
ENERGY_KEY = "HVAC_electricity_demand_rate"

SHORT_RUNPERIOD = (1, 1, 1991, 7, 1, 1991)
TIMESTEPS_PER_HOUR = 1
MAX_EPISODE_STEPS = 48

RUN_TAG = datetime.now().strftime("%Y%m%d-%H%M%S")
REPORT_DIR = os.path.join("reports", RUN_TAG)
CSV_PATH   = os.path.join(REPORT_DIR, "report.csv")
TXT_PATH   = os.path.join(REPORT_DIR, "report.txt")

# -------------------- SOCKET UTILS --------------------
def start_server(host=HOST, port=PORT):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    print(f"[SOCKET] Listening on {host}:{port} ...", flush=True)
    conn, addr = srv.accept()
    print(f"[SOCKET] Connected by {addr}", flush=True)
    f = conn.makefile(mode="rwb", buffering=0)
    return srv, conn, f

def safe_send_float_line(conn, f, value: float):
    line = f"{float(value):.6f}\n"
    print(f"[SOCKET][TX] {line.strip()}", flush=True)
    try:
        f.write(line.encode("utf-8"))
        return True
    except (BrokenPipeError, ConnectionResetError, OSError):
        print("[SOCKET] peer closed while sending; stopping.", flush=True)
        return False

def safe_send_text_line(conn, f, text: str):
    line = text.strip() + "\n"
    print(f"[SOCKET][TX] {line.strip()}", flush=True)
    try:
        f.write(line.encode("utf-8"))
        return True
    except (BrokenPipeError, ConnectionResetError, OSError):
        return False

def _is_mostly_printable(bs: bytes) -> bool:
    # simple heuristic: printable + whitespace ratio
    import string
    printable = set(bytes(string.printable, 'ascii'))
    if not bs: return False
    good = sum(b in printable for b in bs)
    return (good / len(bs)) > 0.8

def _try_parse_text_line(s: str):
    # sanitize
    s = s.replace('\x00', '').replace('\r', '').strip()
    if not s:
        raise ValueError("empty line")
    # allow { } or [ ]
    s = s.replace('{','[').replace('}',']')
    s = s.strip('[]')
    # comma or whitespace
    parts = s.split(',') if (',' in s) else s.split()
    if len(parts) != 2:
        raise ValueError(f"need 2 numbers in text, got: {s!r}")
    spc = float(parts[0].strip())
    sph = float(parts[1].strip())
    return spc, sph, s  # include raw for logging

def _try_parse_binary(bs: bytes):
    # try 16 bytes = two doubles
    if len(bs) < 16: 
        raise ValueError("not enough bytes for 2*doubles")
    cand = bs[:16]
    # little-endian
    try:
        spc, sph = struct.unpack('<dd', cand)
        if math.isfinite(spc) and math.isfinite(sph):
            return spc, sph, f"<binary-le {cand.hex()}>"
    except Exception:
        pass
    # big-endian
    try:
        spc, sph = struct.unpack('>dd', cand)
        if math.isfinite(spc) and math.isfinite(sph):
            return spc, sph, f"<binary-be {cand.hex()}>"
    except Exception:
        pass
    raise ValueError("binary parse failed")

def recv_setpoints_line(conn, f, timeout=SOCKET_TIMEOUT):
    """
    Robust read:
      - Read from socket until '\n' or '\0' (or timeout).
      - If looks like text, parse text.
      - Else try 16-byte binary doubles (little/big endian).
    Returns spcool, spheater
    """
    conn.settimeout(timeout)
    buf = bytearray()
    while True:
        chunk = conn.recv(1024)
        if not chunk:
            # peer closed; if we have something, use it; else raise
            if buf:
                break
            raise ConnectionError("Socket closed by peer.")
        buf.extend(chunk)
        # stop on newline OR NUL
        if b'\n' in chunk or b'\x00' in chunk:
            break
        # safety: avoid unbounded growth; if too long, assume one frame
        if len(buf) > 4096:
            break

    # If we saw a delimiter, cut at first occurrence
    cut = len(buf)
    for d in (b'\n', b'\x00'):
        idx = buf.find(d)
        if idx != -1:
            cut = min(cut, idx)
    frame = bytes(buf[:cut])

    try:
        if _is_mostly_printable(frame):
            s = frame.decode('utf-8', errors='ignore')
            spc, sph, raw = _try_parse_text_line(s)
            print(f"[SOCKET][RX] raw='{raw}' -> spcool={spc:.2f}, spheater={sph:.2f}", flush=True)
            return spc, sph
        else:
            spc, sph, raw = _try_parse_binary(frame)
            print(f"[SOCKET][RX] raw={raw} -> spcool={spc:.2f}, spheater={sph:.2f}", flush=True)
            return spc, sph
    except Exception as e:
        # last attempt: decode ignoring NULs and retry text
        try:
            s = frame.replace(b'\x00', b'').decode('utf-8', errors='ignore')
            spc, sph, raw = _try_parse_text_line(s)
            print(f"[SOCKET][RX] raw='{raw}' -> spcool={spc:.2f}, spheater={sph:.2f}", flush=True)
            return spc, sph
        except Exception:
            pass
        raise

# -------------------- ENV / OBS UTILS --------------------
def obs_to_dict(env, obs):
    if isinstance(obs, dict):
        return {k: float(v) for k, v in obs.items()}
    names = env.get_wrapper_attr('observation_variables') or []
    arr = np.asarray(obs).ravel()
    if len(names) == len(arr):
        return {k: float(x) for k, x in zip(names, arr)}
    return {f"obs_{i}": float(v) for i, v in enumerate(arr)}

def get_from_obs(env, obs, name, default=np.nan):
    d = obs_to_dict(env, obs)
    return float(d.get(name, default))

def clamp_action(env, a):
    return np.clip(a, env.action_space.low, env.action_space.high).astype(env.action_space.dtype)

def write_txt_line(fp, line):
    fp.write(line + "\n"); fp.flush()

# -------------------- MAIN --------------------
def run():
    os.makedirs(REPORT_DIR, exist_ok=True)

    with open(CSV_PATH, "w", newline="") as csvf, open(TXT_PATH, "w") as txtf:
        csvw = csv.writer(csvf)
        csvw.writerow([
            "step","month","day","hour",
            "Tin","Tout","energy",
            "htg_action","clg_action",
            "htg_obs","clg_obs",
            "reward",
            "deadband_ok","in_bounds"
        ])
        write_txt_line(txtf, f"# Sinergym run @ {RUN_TAG}")
        write_txt_line(txtf, "# step | time | Tin | Tout | energy | act(htg,clg) | obsSP(htg,clg) | R | flags")

        srv, conn, f = start_server(HOST, PORT)

        new_time_variables = ['month', 'day_of_month', 'hour']
        new_variables = {
            'outdoor_temperature': ('Site Outdoor Air DryBulb Temperature', 'Environment'),
            'west_zone_htg_setpoint': ('Zone Thermostat Heating Setpoint Temperature', 'West Zone'),
            'west_zone_clg_setpoint': ('Zone Thermostat Cooling Setpoint Temperature', 'West Zone'),
            'west_zone_air_temperature': ('Zone Air Temperature', 'West Zone'),
            'HVAC_electricity_demand_rate': ('Facility Total HVAC Electricity Demand Rate', 'Whole Building'),
        }
        new_meters = {}
        new_actuators = {
            'Heating_Setpoint_RL': ('Schedule:Compact', 'Schedule Value', 'Heating Setpoints'),
            'Cooling_Setpoint_RL': ('Schedule:Compact', 'Schedule Value', 'Cooling Setpoints'),
        }
        new_action_space = gym.spaces.Box(
            low=np.array([14.0, 22.0], dtype=np.float32),
            high=np.array([22.0, 30.5], dtype=np.float32),
            shape=(2,), dtype=np.float32
        )
        building_config = {'runperiod': SHORT_RUNPERIOD, 'timesteps_per_hour': TIMESTEPS_PER_HOUR}

        env = gym.make(
            'Eplus-datacenter-cool-continuous-stochastic-v1',
            time_variables=new_time_variables,
            variables=new_variables,
            meters=new_meters,
            actuators=new_actuators,
            action_space=new_action_space,
            building_config=building_config,
        )
        env = TimeLimit(env, max_episode_steps=MAX_EPISODE_STEPS)

        try:
            obs, info = env.reset()
            terminated = truncated = False
            step = 0
            rewards = []

            while not (terminated or truncated):
                month = int(info.get('month', -1))
                day   = int(info.get('day_of_month', -1))
                hour  = int(info.get('hour', -1))

                Tin    = get_from_obs(env, obs, T_ROOM_KEY, np.nan)
                Tout   = get_from_obs(env, obs, T_OUT_KEY,  np.nan)
                energy = get_from_obs(env, obs, ENERGY_KEY, np.nan)

                if not safe_send_float_line(conn, f, Tin):
                    break

                try:
                    spcool, spheater = recv_setpoints_line(conn, f, timeout=SOCKET_TIMEOUT)
                except Exception as e:
                    low, high = env.action_space.low, env.action_space.high
                    spheater = float((low[0] + high[0]) * 0.5)
                    spcool   = float((low[1] + high[1]) * 0.5)
                    print(f"[SOCKET][RX] fallback -> spcool={spcool:.2f}, spheater={spheater:.2f} (reason: {e})", flush=True)

                htg = float(spheater)
                clg = float(spcool)
                clg = max(clg, htg + DEADBAND)
                action = clamp_action(env, np.array([htg, clg], dtype=np.float32))

                obs, reward, terminated, truncated, info = env.step(action)
                rewards.append(reward)

                h_sp = get_from_obs(env, obs, 'west_zone_htg_setpoint', np.nan)
                c_sp = get_from_obs(env, obs, 'west_zone_clg_setpoint', np.nan)

                deadband_ok = (action[1] >= action[0] + DEADBAND - 1e-6)
                in_bounds   = (
                    (action[0] >= env.action_space.low[0] - 1e-6) and
                    (action[0] <= env.action_space.high[0] + 1e-6) and
                    (action[1] >= env.action_space.low[1] - 1e-6) and
                    (action[1] <= env.action_space.high[1] + 1e-6)
                )

                line = (f"[{step:04d}] {month:02d}/{day:02d} {hour:02d}h | "
                        f"Tin={Tin:.2f} Tout={Tout:.2f} E={energy:.2f} | "
                        f"act=[htg={action[0]:.2f}, clg={action[1]:.2f}] | "
                        f"obsSP=[htg={h_sp:.2f}, clg={c_sp:.2f}] | "
                        f"R={reward:.3f} | flags: deadband={deadband_ok} bounds={in_bounds}")
                print(line, flush=True)
                write_txt_line(txtf, line)

                csvw.writerow([
                    step, month, day, hour,
                    f"{Tin:.4f}", f"{Tout:.4f}", f"{energy:.6f}",
                    f"{action[0]:.4f}", f"{action[1]:.4f}",
                    f"{h_sp:.4f}", f"{c_sp:.4f}",
                    f"{reward:.6f}",
                    int(deadband_ok), int(in_bounds)
                ])

                step += 1

            if rewards:
                summary = (f"Episode done. Steps={step}, "
                           f"MeanR={np.mean(rewards):.4f}, CumR={np.sum(rewards):.4f}")
            else:
                summary = "Episode ended early (no rewards collected)."
            print("\n" + summary, flush=True)
            write_txt_line(txtf, "# " + summary)

            safe_send_text_line(conn, f, "DONE")

        finally:
            try: f.close()
            except: pass
            try: conn.close()
            except: pass
            try: srv.close()
            except: pass
            env.close()

if __name__ == "__main__":
    run()
