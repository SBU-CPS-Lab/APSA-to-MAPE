# sinergym_socket_server_clean.py

import os
import csv
import socket
from dataclasses import dataclass
from datetime import datetime

import gymnasium as gym
import numpy as np
import sinergym
from gymnasium.wrappers import TimeLimit


@dataclass(frozen=True)
class Config:
    host: str = "0.0.0.0"
    port: int = 9000
    timeout_s: float = 30.0

    deadband: float = 1.0

    # Sinergym
    timesteps_per_hour: int = 1
    max_episode_steps: int = 48
    runperiod: tuple = (1, 1, 1991, 7, 1, 1991)

    # Observation keys
    t_room_key: str = "west_zone_air_temperature"
    t_out_key: str = "outdoor_temperature"
    energy_key: str = "HVAC_electricity_demand_rate"


class SocketProtocol:
    """
    Simple protocol that matches your C++ socketwrap:
      - messages are UTF-8 text terminated with NUL ('\\x00')
    """
    def __init__(self, conn: socket.socket, timeout_s: float):
        self.conn = conn
        self.conn.settimeout(timeout_s)

    def send_cstring(self, text: str) -> None:
        msg = (text + "\x00").encode("utf-8")
        self.conn.sendall(msg)

    def recv_cstring(self) -> str:
        buf = bytearray()
        while True:
            chunk = self.conn.recv(1024)
            if not chunk:
                raise ConnectionError("Socket closed by peer.")
            buf.extend(chunk)
            nul = buf.find(b"\x00")
            if nul != -1:
                return buf[:nul].decode("utf-8", errors="ignore").strip()

    @staticmethod
    def parse_two_floats(s: str) -> tuple[float, float]:
        """
        Accept:
          "a,b" or "a b" or "[a,b]" or "{a,b}"
        Returns: (spcool, spheater)
        """
        s = s.strip()
        s = s.replace("{", "[").replace("}", "]").strip("[]")
        parts = s.split(",") if "," in s else s.split()
        if len(parts) != 2:
            raise ValueError(f"Expected 2 floats, got: {s!r}")
        return float(parts[0]), float(parts[1])


class Runner:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.run_tag = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.report_dir = os.path.join("reports", self.run_tag)
        os.makedirs(self.report_dir, exist_ok=True)

        self.csv_path = os.path.join(self.report_dir, "report.csv")
        self.txt_path = os.path.join(self.report_dir, "report.txt")

    def _start_server(self) -> tuple[socket.socket, socket.socket]:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.cfg.host, self.cfg.port))
        srv.listen(1)
        print(f"[SOCKET] Listening on {self.cfg.host}:{self.cfg.port} ...", flush=True)

        conn, addr = srv.accept()
        print(f"[SOCKET] Connected by {addr}", flush=True)
        return srv, conn

    def _make_env(self):
        new_time_variables = ["month", "day_of_month", "hour"]
        new_variables = {
            "outdoor_tempersature": ("Site Outdoor Air DryBulb Temperature", "Environment"),
            "west_zone_htg_setpoint": ("Zone Thermostat Heating Setpoint Temperature", "West Zone"),
            "west_zone_clg_setpoint": ("Zone Thermostat Cooling Setpoint Temperature", "West Zone"),
            "west_zone_air_temperature": ("Zone Air Temperature", "West Zone"),
            "HVAC_electricity_demand_rate": ("Facility Total HVAC Electricity Demand Rate", "Whole Building"),
        }
        new_actuators = {
            "Heating_Setpoint_RL": ("Schedule:Compact", "Schedule Value", "Heating Setpoints"),
            "Cooling_Setpoint_RL": ("Schedule:Compact", "Schedule Value", "Cooling Setpoints"),
        }
        new_action_space = gym.spaces.Box(
            low=np.array([14.0, 22.0], dtype=np.float32),
            high=np.array([22.0, 30.5], dtype=np.float32),
            shape=(2,),
            dtype=np.float32,
        )
        building_config = {"runperiod": self.cfg.runperiod, "timesteps_per_hour": self.cfg.timesteps_per_hour}

        env = gym.make(
            "Eplus-datacenter_dx-cool-continuous-stochastic-v1",
            time_variables=new_time_variables,
            variables=new_variables,
            meters={},
            actuators=new_actuators,
            action_space=new_action_space,
            building_config=building_config,
        )
        return TimeLimit(env, max_episode_steps=self.cfg.max_episode_steps)

    @staticmethod
    def _obs_to_dict(env, obs) -> dict:
        if isinstance(obs, dict):
            return {k: float(v) for k, v in obs.items()}
        names = env.get_wrapper_attr("observation_variables") or []
        arr = np.asarray(obs).ravel()
        if len(names) == len(arr):
            return {k: float(x) for k, x in zip(names, arr)}
        return {}

    @staticmethod
    def _clamp_action(env, action: np.ndarray) -> np.ndarray:
        return np.clip(action, env.action_space.low, env.action_space.high).astype(env.action_space.dtype)

    def run(self):
        srv, conn = self._start_server()
        proto = SocketProtocol(conn, timeout_s=self.cfg.timeout_s)
        env = self._make_env()

        try:
            with open(self.csv_path, "w", newline="") as csvf, open(self.txt_path, "w") as txtf:
                csvw = csv.writer(csvf)
                csvw.writerow(["step", "month", "day", "hour", "Tin", "Tout", "energy", "htg", "clg", "reward"])

                obs, info = env.reset()
                terminated = truncated = False
                step = 0

                while not (terminated or truncated):
                    month = int(info.get("month", -1))
                    day = int(info.get("day_of_month", -1))
                    hour = int(info.get("hour", -1))

                    d = self._obs_to_dict(env, obs)
                    Tin = float(d.get(self.cfg.t_room_key, np.nan))
                    Tout = float(d.get(self.cfg.t_out_key, np.nan))
                    energy = float(d.get(self.cfg.energy_key, np.nan))

                    # 1) Send Tin to ForSyDe
                    proto.send_cstring(f"{Tin:.6f}")

                    # 2) Receive setpoints from ForSyDe
                    msg = proto.recv_cstring()
                    spcool, spheater = proto.parse_two_floats(msg)

                    # 3) Apply deadband and clamp
                    htg = float(spheater)
                    clg = max(float(spcool), htg + self.cfg.deadband)
                    action = self._clamp_action(env, np.array([htg, clg], dtype=np.float32))

                    # 4) Step env
                    obs, reward, terminated, truncated, info = env.step(action)

                    line = (
                        f"[{step:04d}] {month:02d}/{day:02d} {hour:02d} "
                        f"Tin={Tin:.2f} Tout={Tout:.2f} E={energy:.2f} "
                        f"act=[{action[0]:.2f},{action[1]:.2f}] R={float(reward):.3f}"
                    )
                    print(line, flush=True)
                    txtf.write(line + "\n")
                    txtf.flush()

                    csvw.writerow([
                        step, month, day, hour,
                        f"{Tin:.4f}", f"{Tout:.4f}", f"{energy:.6f}",
                        f"{action[0]:.4f}", f"{action[1]:.4f}", f"{float(reward):.6f}",
                    ])

                    step += 1

                proto.send_cstring("DONE")

        finally:
            try: conn.close()
            except: pass
            try: srv.close()
            except: pass
            env.close()


if __name__ == "__main__":
    Runner(Config()).run()
