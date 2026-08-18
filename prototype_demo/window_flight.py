"""창문 통과 비행 — 계획 → 궤적 → **성진 제어기** → PyBullet 물리.

    python window_flight.py                     # GT 창문으로 (제어만 검증)
    python window_flight.py --vision            # 비전 복원 창문으로 (전 구간)

제어기에는 **pose 만** 준다 — 그의 QcInput 에 속도 필드가 없다.
설정은 control/qc.py 의 PYBULLET_TUNED (전수조사로 찾은 모터 배정 + 튜닝 게인).
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from utils import paths  # noqa: E402

paths.bootstrap()

import planner  # noqa: E402
import traj  # noqa: E402
from control import qc  # noqa: E402
from module import contract  # noqa: E402
from overrides import detections as ovd  # noqa: E402
from overrides import frames as ovf  # noqa: E402
from overrides import recon_rays  # noqa: E402
from utils import device, metrics  # noqa: E402
from utils import scene, viz  # noqa: E402


def gt_window_map(layout):
    return [{"order_index": w["order_index"], "color": w["color"],
             "center": list(map(float, w["center"])),
             "corners_3d": scene.corners_world(w).tolist(),
             "size_wh": [w["ow"], w["oh"]]} for w in layout]


def passage_report(path: np.ndarray, layout, margin: float):
    """궤적이 각 창문 평면을 어디서 통과했는지. 반환 rows."""
    UP = np.array([0.0, 0.0, 1.0])
    rows = []
    for w in layout:
        c = np.asarray(w["center"], float)
        n = np.array([-1.0, 0.0, 0.0])                 # 창문 법선 (씬 규약: -x)
        wa = np.cross(UP, n); wa /= np.linalg.norm(wa)
        d = (path - c) @ n
        hit = None
        for k in range(len(d) - 1):
            if d[k] * d[k + 1] < 0:
                a = d[k] / (d[k] - d[k + 1])
                hit = path[k] + a * (path[k + 1] - path[k])
                break
        if hit is None:
            rows.append({"color": w["color"], "passed": False})
            continue
        du = float((hit - c) @ wa)
        dv = float((hit - c) @ UP)
        hw, hh = w["ow"] / 2, w["oh"] / 2
        rows.append({"color": w["color"], "passed": True, "u": du, "v": dv,
                     "slack_u": hw - abs(du), "slack_v": hh - abs(dv),
                     "ok": abs(du) <= hw - margin and abs(dv) <= hh - margin})
    return rows


def _write_pair(g3, g1, a) -> None:
    """3인칭 / 1인칭 GIF 를 따로 쓴다. 경로는 --gif 에 _3p / _fpv 를 붙인다."""
    root, ext = os.path.splitext(a.gif)
    for frames, suf, what in ((g3, "_3p", "3인칭 고정 카메라"),
                              (g1, "_fpv", "1인칭 드론 카메라")):
        if frames:
            _write_gif(frames, a, f"{root}{suf}{ext or '.gif'}", what)


def _write_gif(gif_frames, a, path=None, what="") -> None:
    from PIL import Image
    path = path or a.gif
    dur = max(30, int(round(a.gif_seconds * 1000 / len(gif_frames))))
    gw = 900
    if gif_frames[0].width > gw:
        hh2 = round(gif_frames[0].height * gw / gif_frames[0].width)
        gif_frames = [x.resize((gw, hh2), Image.LANCZOS) for x in gif_frames]
    pal = gif_frames[0].quantize(colors=112, method=Image.MEDIANCUT)
    gf = [x.quantize(palette=pal, dither=Image.FLOYDSTEINBERG) for x in gif_frames]
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    gf[0].save(path, save_all=True, append_images=gf[1:], duration=dur,
               loop=0, optimize=True)
    print(f"wrote {path}  {what}  ({len(gf)} frames, {len(gf)*dur/1000:.1f} s, "
          f"{os.path.getsize(path)/2**20:.1f} MB)")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--n-windows", type=int, default=3)
    ap.add_argument("--spacing", type=float, default=2.6,
                    help="2.5 미만이면 게이트가 겹쳐 후진이 생긴다")
    ap.add_argument("--opening", type=float, default=1.0)
    ap.add_argument("--clutter", type=int, default=10)
    ap.add_argument("--dt", type=float, default=0.001)
    ap.add_argument("--v-frac", type=float, default=0.6)
    ap.add_argument("--smooth", type=float, default=0.25)
    ap.add_argument("--spin-up", type=float, default=1.5)
    ap.add_argument("--traj", default="seoungjin",
                    choices=("seoungjin", "mine", "capped"),
                    help="궤적 생성기. seoungjin=path_time 7차 다항식 최소시간, "
                         "capped=곡률제한 폴리라인 (후진 없음)")
    ap.add_argument("--merge-m", type=float, default=0.12,
                    help="seoungjin 생성기 웨이포인트 중복제거 반경 [m]")
    ap.add_argument("--gif", default=None,
                    help="비행 GIF 경로. 좌=고정카메라+화각, 우=드론시점+검출")
    ap.add_argument("--gif-every", type=int, default=90)
    ap.add_argument("--gif-seconds", type=float, default=22.0)
    ap.add_argument("--seq", action="store_true",
                    help="순차 통과 모드: 목표 색만 마스킹 + 최대 박스 1개로 "
                         "창문을 하나씩 비전으로 찾아 통과. GT 계획 안 씀")
    ap.add_argument("--replan", action="store_true",
                    help="복도를 조금 날며 관측 -> 비전으로 재계획 -> 나머지 통과. "
                         "GT 창문을 안 쓰는 전 구간 모드")
    ap.add_argument("--trust-parallax", type=float, default=15.0,
                    help="이 시차각[deg] 이상인 창문만 계획에 넣는다. "
                         "시차가 낮으면 복원이 못 믿을 값이다 (실측: 5.7deg -> 2825mm, "
                         "19.3deg -> 6mm)")
    ap.add_argument("--max-legs", type=int, default=6)
    ap.add_argument("--lead-m", type=float, default=1.6,
                    help="--replan 의 1구간 전진 거리 [m]")
    ap.add_argument("--scan-y", type=float, default=0.65,
                    help="--seq 탐색의 가로 오프셋 [m]. 시차는 옆으로 움직여야 "
                         "벌린다 — 정면 직진은 방위각이 안 변해 시차가 안 쌓인다")
    ap.add_argument("--scan-z", type=float, default=0.35,
                    help="--seq 탐색의 세로 오프셋 [m]")
    ap.add_argument("--align-back", type=float, default=1.1,
                    help="--seq 통과 전 법선 정렬점을 접근 게이트보다 얼마나 뒤에 "
                         "둘지 [m]. 탐색 오프셋에서 곧장 들어가면 가로로 못 붙는다")
    ap.add_argument("--max-probe", type=int, default=4,
                    help="--seq 창문 하나당 탐색 시도 횟수")
    ap.add_argument("--export-euroc", default=None,
                    help="EuRoC MAV(MH) 포맷으로 덤프할 경로. IMU 200Hz + 카메라 "
                         "20Hz + GT, 나노초 시계 공유")
    ap.add_argument("--imu-hz", type=float, default=200.0)
    ap.add_argument("--cam-hz", type=float, default=20.0)
    ap.add_argument("--static-s", type=float, default=3.0,
                    help="비행 전 정지 구간 [s]. OpenVINS 정적 초기화가 중력 방향과 "
                         "자이로 바이어스를 잡으려면 정지 상태가 필요하다 "
                         "(--export-euroc 일 때만 기록된다)")
    ap.add_argument("--no-noise", action="store_true",
                    help="IMU 노이즈 끄기 (기본은 EuRoC ADIS16448 스펙 적용)")
    ap.add_argument("--observe", action="store_true",
                    help="비행하면서 카메라로 관측 + 창문 복원 (GT 계획은 그대로)")
    ap.add_argument("--weights", default=None)
    ap.add_argument("--render-every", type=int, default=60)
    ap.add_argument("--render-scale", type=float, default=0.5)
    ap.add_argument("--det-conf-min", type=float, default=0.5)
    ap.add_argument("--device", default="auto", choices=("auto", "cuda", "cpu"))
    ap.add_argument("--fig", default=os.path.join(paths.FIG_DIR, "v2_09_flight.png"))
    a = ap.parse_args(argv)

    import pybullet as p
    from window_waypoint_planner import load_planner_config
    from sim import pybullet_stream as pbs
    from eval_recon3d import quat_xyzw_to_rot

    ovf.install()          # T_imu_cam 을 표준 전방 카메라 값으로 (overrides/frames.py)
    cfg = load_planner_config(paths.PLANNER_LIMITS)
    margin = float(cfg["clearance_margin"])

    env, layout = scene.make(seed=a.seed, n_windows=a.n_windows, clutter=a.clutter,
                             opening=a.opening, spacing=a.spacing)
    cid = env.CLIENT
    scene.print_layout(layout)
    xs = [float(w["center"][0]) for w in layout]
    print(f"간격 {[f'{g:.2f}' for g in np.diff(xs)]}  "
          f"(standoff 요구 {scene.GATE_STANDOFF_M} m)")

    # --- 계획 ---------------------------------------------------------------
    pl = planner.plan(gt_window_map(layout), start=(0.0, 0.0, 1.0), cfg=cfg)
    print(f"\n[계획] {planner.describe(pl)}")

    # --- 궤적 ---------------------------------------------------------------
    if a.traj == "seoungjin":
        sys.path.insert(0, paths.CONTROL)
        t, ref, refv, L, T = traj.build_seoungjin(pl.waypoints, cfg, dt=a.dt,
                                                  merge_m=a.merge_m)
        src = "성진 path_time.plan_waypoints_flythrough (7차 다항식 최소시간)"
    elif a.traj == "capped":
        t, ref, refv, L, T = traj.build_capped(pl.waypoints, cfg, dt=a.dt)
        src = "traj.build_capped (곡률제한 폴리라인 — 후진 없음)"
    else:
        t, ref, refv, L, T = traj.build(pl.waypoints, dt=a.dt,
                                        v_max=cfg["limits"]["v_max"],
                                        v_frac=a.v_frac, smooth_m=a.smooth)
        src = "traj.build (smoothstep + 이동평균)"
    va = np.abs(refv).max(axis=0)
    print(f"[궤적] {src}")
    print(f"       길이 {L:.2f} m, {T:.2f} s + 정지 유지,  축별 최대속도 "
          f"[{va[0]:.2f} {va[1]:.2f} {va[2]:.2f}] (v_max {cfg['limits']['v_max']})")

    # --- 제어기 + 기체 -------------------------------------------------------
    p.setTimeStep(a.dt, physicsClientId=cid)
    ctl = qc.Controller(alt_cmd_sat=qc.PYBULLET_TUNED["alt_cmd_sat"]).apply_tuned()
    body = qc.make_body(p, cid, ctl.m_tot, ctl.I_att, ctl.I_yaw, ctl.r_arm,
                        start=tuple(ref[0]))
    # env 자체 CF2X 는 이 실험에 안 쓴다 — 멀리 치워 충돌만 피한다
    p.resetBasePositionAndOrientation(env.DRONE_IDS[0], [0, 0, -50], [0, 0, 0, 1],
                                      physicsClientId=cid)
    print(f"[기체] m {ctl.m_tot:.3f} kg  T/W {ctl.thrust_to_weight():.2f}  "
          f"반경 {ctl.r_arm:.3f} m  (개구부 반폭 {a.opening/2:.2f} m, 여유 {margin} m)")

    det = (device.load_detector(a.weights, prefer=a.device)
       if (a.observe or a.seq or a.replan) else None)
    intr = contract.intrinsics()
    T_IC = contract.T_imu_cam()
    samples = []
    rec = None
    if a.export_euroc:
        from export_euroc import EurocRecorder
        rec = EurocRecorder(T_IC=contract.T_imu_cam(), intr=intr, dt=a.dt,
                            imu_hz=a.imu_hz, cam_hz=a.cam_hz,
                            noise=not a.no_noise, seed=a.seed)
        print(f"[EuRoC] IMU {a.imu_hz:.0f} Hz / 카메라 {a.cam_hz:.0f} Hz  "
              f"노이즈 {'ON' if rec.noise else 'OFF'}  -> {a.export_euroc}")
    gif_3p, gif_fpv = [], []      # 3인칭(고정 카메라) / 1인칭(드론 카메라)
    _mid = np.mean([w["center"] for w in layout], axis=0)
    _W, _H = 900, 506
    _view = _proj = None

    def shot_pair(pos, quat):
        """좌: 고정 카메라 + 화각 프러스텀 + 궤적 / 우: 드론 시점 + 검출."""
        from PIL import Image, ImageDraw, ImageEnhance
        from utils.viz import cam_axes, project
        nonlocal _view, _proj
        if _view is None:
            _view = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=_mid.tolist(), distance=8.0, yaw=48,
                pitch=-24, roll=0, upAxisIndex=2, physicsClientId=cid)
            _proj = p.computeProjectionMatrixFOV(60, _W / _H, 0.05, 60,
                                                 physicsClientId=cid)
        _, _, rgb, _, _ = p.getCameraImage(_W, _H, _view, _proj, shadow=1,
                                           renderer=p.ER_TINY_RENDERER,
                                           physicsClientId=cid)
        im = Image.fromarray(np.asarray(rgb, np.uint8).reshape(_H, _W, 4)[:, :, :3])
        im = ImageEnhance.Brightness(im).enhance(1.7)
        dr = ImageDraw.Draw(im)
        tp = [q for q in project(np.array([x for x, _ in gif_trail]), _view, _proj,
                                 _W, _H) if q] if gif_trail else []
        if len(tp) > 1:
            dr.line(tp, fill=(255, 210, 60), width=2)
        R_WB = quat_xyzw_to_rot(np.asarray(quat, float))
        yaw = float(np.arctan2(R_WB[1, 0], R_WB[0, 0]))
        hh = float(np.degrees(np.arctan(intr["width"] / 2 / intr["fx"])))
        hv = float(np.degrees(np.arctan(intr["height"] / 2 / intr["fy"])))
        f, r_, u_ = cam_axes(yaw)
        th_, tv_ = np.tan(np.radians(hh)), np.tan(np.radians(hv))
        fp = np.array([np.asarray(pos) + 3.0 * (f - th_*r_ + tv_*u_),
                       np.asarray(pos) + 3.0 * (f + th_*r_ + tv_*u_),
                       np.asarray(pos) + 3.0 * (f + th_*r_ - tv_*u_),
                       np.asarray(pos) + 3.0 * (f - th_*r_ - tv_*u_)])
        s0 = project(np.array([pos]), _view, _proj, _W, _H)[0]
        se = project(fp, _view, _proj, _W, _H)
        if s0 and all(se):
            for e_ in se:
                dr.line([s0, e_], fill=(255, 255, 255), width=1)
            dr.line(list(se) + [se[0]], fill=(255, 255, 255), width=2)
            dr.ellipse([s0[0]-7, s0[1]-7, s0[0]+7, s0[1]+7],
                       outline=(255, 255, 255), width=3)
        dr.text((8, 8), f"fixed camera   drone ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})",
                fill=(255,)*3)
        dr.text((8, 24), f"FOV  h +/-{hh:.0f} deg  v +/-{hv:.0f} deg", fill=(255,)*3)
        gif_trail.append((np.asarray(pos, float), 0))

        fwd = R_WB[:, 0]
        img2 = pbs.render_frame(p, cid, np.asarray(pos, float),
                                np.asarray(pos, float) + fwd, intr, scale=0.5)
        im2 = Image.fromarray(img2)
        dr2 = ImageDraw.Draw(im2)
        got = []
        if det is not None:
            rr = det.predict(img2[:, :, ::-1], conf=0.25, agnostic_nms=True)[0]
            if rr.boxes is not None and len(rr.boxes):
                COL = {"red": (235, 60, 60), "green": (40, 200, 60),
                       "blue": (60, 130, 235)}
                for bx, kp in zip(rr.boxes, rr.keypoints.xy.cpu().numpy()):
                    nm = det.names[int(bx.cls)]; c_ = COL.get(nm, (255,)*3)
                    pl_ = [tuple(map(float, t)) for t in kp]
                    dr2.line(pl_ + [pl_[0]], fill=c_, width=3)
                    got.append(f"{nm} {float(bx.conf):.2f}")
        dr2.text((8, 8), "drone camera + detections", fill=(255,)*3)
        dr2.text((8, 24), "  ".join(got) if got else "NO DETECTION",
                 fill=(120, 255, 120) if got else (255, 90, 90))
        return im, im2          # (3인칭 고정 카메라, 1인칭 드론 카메라)

    gif_trail = []

    def observe_now(t_ns, fid):
        """현재 자세에서 카메라 1장 → §5 샘플. 비행 로직은 건드리지 않는다."""
        ps, qt = p.getBasePositionAndOrientation(body, physicsClientId=cid)
        R_WB = quat_xyzw_to_rot(np.asarray(qt, float))
        fwd = R_WB[:, 0]                       # 카메라는 body +x 를 본다
        p_WC, q_WC = pbs.look_at_pose(np.asarray(ps, float),
                                      np.asarray(ps, float) + fwd)
        img = pbs.render_frame(p, cid, p_WC, np.asarray(ps, float) + fwd, intr,
                               scale=a.render_scale)
        msg = pbs.infer_frame_multiclass(det, img, t_ns, fid, 0.25)
        R_WC = quat_xyzw_to_rot(np.asarray(q_WC, float))
        R_WI, p_WI = contract.camera_pose_to_imu_pose(R_WC, p_WC, T_IC)
        return {"t_ns": t_ns, "p_WI": [float(v) for v in p_WI],
                "q_WI_xyzw": [float(v) for v in pbs._rot_to_quat_xyzw(R_WI)],
                "detection": msg if msg["windows"] else None}

    if rec is not None and a.static_s > 0:
        # OpenVINS 정적 초기화용 정지 구간. 자세/속도를 매 스텝 고정해 완전 정지로
        # 만든다 (자이로 0, 가속도계는 중력만). 노이즈는 그대로 얹힌다.
        n_st = int(a.static_s / a.dt)
        ps0, q0 = p.getBasePositionAndOrientation(body, physicsClientId=cid)
        R0 = quat_xyzw_to_rot(np.asarray(q0, float))
        print(f"[정지 구간] {a.static_s:.1f} s 기록 — OpenVINS 정적 초기화용")
        for k_ in range(n_st):
            ctl.step(ps0, 0.0, ps0, (0, 0, 0), a.dt)
            p.resetBasePositionAndOrientation(body, ps0, q0, physicsClientId=cid)
            p.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0], physicsClientId=cid)
            img_e = None
            if k_ % rec.cam_every == 0:
                img_e = pbs.render_frame(p, cid, np.asarray(ps0, float),
                                         np.asarray(ps0, float) + R0[:, 0], intr,
                                         scale=a.render_scale)
            rec.step(p, body, cid, image=img_e)

    gif_k = [0]          # 구간을 넘나드는 전역 스텝 카운터 (GIF 간격용)

    #: 다음 구간 궤적의 **시작점**. 측정 위치가 아니라 직전 구간 ref 의 종점이다.
    ref_end = np.array(p.getBasePositionAndOrientation(body, physicsClientId=cid)[0])

    def fly_leg(W, observe=True, spin=False):
        """웨이포인트 -> 궤적 -> 비행.

        구간을 나눌 때 지켜야 하는 조건은 정지-정지가 아니라 **레퍼런스 연속성**이다.

        측정으로 재본 것 (빈 씬, 성진 제어기, dt=1 ms)
        ---------------------------------------------
        끊김 없는 8 m 단일 궤적 : 오차 22 mm 로 완주
        같은 궤적 한가운데에 z 를 **10 mm** 만 툭 내림 : 4.08 s 에 tilt 61° 발산
        같은 자리에 42 mm : 총추력이 44.5 N (= T/W 2.0 최대치) 에 붙어 안 떨어지고
                            z 가 4 m 까지 솟음 (구간 경계에서 각속도 일정 = 차동
                            여유가 0, 즉 자세 권한 상실)

        성진 PID 의 D 항은 후방차분 ``(e - ePrev)/dt`` 다 (qc_controller.hpp:52).
        dt = 1 ms 에서 10 mm 스텝은 **10 m/s** 로 읽힌다. 위치 스텝은 눈에 안 보여도
        미분으로는 폭탄이다.

        그래서 새 구간은 드론의 **측정 위치**가 아니라 **직전 구간 ref 의 종점**에서
        잇는다. 측정 위치로 시작하면 매 구간 경계에 "추종오차 크기의 스텝" 을 손수
        주입하는 꼴이 된다 (실측 42 mm). 이 한 줄이 구간 분할 사망의 원인이었다.
        실측: 1.6 m x 5 구간이 구간2 에서 죽던 것이 → 5구간 전부 생존,
        추종오차 42 → 23 → 11 → 3 → 7 mm 로 오히려 수렴.
        """
        nonlocal ref_end
        pos_now = np.array(p.getBasePositionAndOrientation(body,
                                                           physicsClientId=cid)[0])
        drift = float(np.linalg.norm(pos_now - ref_end))
        if drift > 0.30:
            # ref 를 그대로 이으면 30 cm 넘는 오차를 한 번에 물린다. 그 경우에만
            # 측정 위치로 재동기화한다 — 스텝은 생기지만 대안이 더 나쁘다.
            print(f"         (ref 이탈 {drift*1000:.0f} mm — 측정 위치로 재동기화)")
            ref_end = pos_now
        WW = np.vstack([ref_end, np.atleast_2d(W)])
        if a.traj == "seoungjin":
            tt, rr, vv, LL, TT = traj.build_seoungjin(WW, cfg, dt=a.dt)
        else:
            tt, rr, vv, LL, TT = traj.build(WW, dt=a.dt,
                                            v_max=cfg["limits"]["v_max"],
                                            v_frac=a.v_frac, smooth_m=a.smooth)
        if spin:
            q0 = p.getBasePositionAndOrientation(body, physicsClientId=cid)[1]
            for _ in range(int(a.spin_up / a.dt)):
                ctl.step(rr[0], 0.0, pos_now, (0, 0, 0), a.dt)
                p.resetBasePositionAndOrientation(body, pos_now, q0,
                                                  physicsClientId=cid)
                p.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0], physicsClientId=cid)
        pth = np.zeros((len(rr), 3)); ok = True
        for k in range(len(rr)):
            ps, qt = p.getBasePositionAndOrientation(body, physicsClientId=cid)
            rp = p.getEulerFromQuaternion(qt)
            th, dq = ctl.step(rr[k], 0.0, ps, rp, a.dt)
            qc.apply_to_body(p, body, th, dq, ctl.motor_xy, ctl.mix_dir, cid)
            p.stepSimulation(physicsClientId=cid)
            pth[k] = ps
            if det is not None and observe and k % a.render_every == 0:
                samples.append(observe_now(len(samples) * 33_000_000, len(samples)))
            if a.gif and (gif_k[0] % a.gif_every == 0):
                _f3, _f1 = shot_pair(ps, qt)
                gif_3p.append(_f3); gif_fpv.append(_f1)
            gif_k[0] += 1
            if (not np.all(np.isfinite(ps)) or ps[2] < 0.25 or ps[2] > 4.0
                    or max(abs(np.degrees(rp[0])), abs(np.degrees(rp[1]))) > 60):
                print(f"  [실패] k={k} z={ps[2]:.2f} "
                      f"tilt={max(abs(np.degrees(rp[0])),abs(np.degrees(rp[1]))):.0f}")
                pth = pth[:k+1]; ok = False; break
        if ok:
            ref_end = rr[-1].copy()
        return pth, ok

    if a.seq:
        print("\n=== 순차 통과 모드 (GT 계획 미사용) ===")
        print("    색 마스킹 + 같은 색 중 최대 박스 1개만\n")
        cur = np.array(p.getBasePositionAndOrientation(body,
                                                       physicsClientId=cid)[0])
        legs = [(np.array([cur]), True)]
        ok_all = True
        rlog = []
        # 창문이 3개를 넘으면 색이 r,g,b 로 순환한다 (10개면 red 가 #0,3,6,9).
        # 최대 박스 1개 규칙은 "지금 화각에서 가장 가까운 동색" 을 고르므로 옳지만,
        # **누적 샘플 전체**에 걸면 예전 프레임에서는 #0 이 가장 컸다는 이유로
        # #3 의 광선 바구니에 #0 의 광선이 섞인다. 그래서 목표 창문마다
        # "직전 창문 통과를 시작한 시점 이후" 의 샘플만 쓴다. 그 시점이면 드론은
        # 이미 같은 색 직전 창문(#k-3)을 지났으므로 가장 큰 동색 = 목표 창문이다.
        s_mark = 0
        for k in range(len(layout)):
            col = ovd.target_color(k)
            # 1) 목표 창문이 보이는 데까지 전진하며 관측
            cur = np.array(p.getBasePositionAndOrientation(body,
                                                           physicsClientId=cid)[0])
            # 시차는 **옆으로** 움직여야 벌린다. 정면 창문을 향해 직진하면 방위각이
            # 거의 안 변해서 시차가 안 쌓이고, 그러는 동안 창문에 박는다
            # (실측: 1.6 m 직진 2회 -> 시차 문턱 못 넘고 x=2.8 에서 red 창틀 충돌).
            # 그래서 탐색은 가로/세로 오프셋을 우선하고, 전진은 창문 위치를
            # 알아낸 뒤 standoff 를 남기고서만 한다.
            anchor = cur.copy()
            probes = [np.array([0.0, +a.scan_y, 0.0]),
                      np.array([0.0, -a.scan_y, 0.0]),
                      np.array([0.0, +a.scan_y, +a.scan_z]),
                      np.array([0.0, -a.scan_y, -a.scan_z])]
            probe = 0
            while probe < a.max_probe:
                masked = ovd.mask_samples(samples[s_mark:], col, k)
                res = recon_rays.reconstruct(masked, det_conf_min=a.det_conf_min)
                par = res[0]["min_parallax_deg"] if res else float("nan")
                nobs = res[0]["n_obs"] if res else 0
                if res and par >= a.trust_parallax:
                    break
                off = probes[probe % len(probes)]
                tgt = anchor + off
                # 전진 여유: 복원이 거칠어도 중심 x 는 쓸 만하다. standoff 를 남긴다.
                if res:
                    x_lim = float(res[0]["center_w"][0]) - cfg["d_app"]
                    tgt[0] += float(np.clip(x_lim - tgt[0], 0.0, a.lead_m))
                tgt[2] = float(np.clip(tgt[2], 0.6, 1.9))
                print(f"[창문 {k} {col}] 시차 {par:5.1f}deg (관측 {nobs}) 부족 "
                      f"-> 탐색 {probe+1}: dy={tgt[1]-anchor[1]:+.2f} "
                      f"dz={tgt[2]-anchor[2]:+.2f} dx={tgt[0]-anchor[0]:+.2f}")
                legs.append(fly_leg(tgt, spin=(k == 0 and probe == 0)))
                ok_all = legs[-1][1]
                cur = np.array(p.getBasePositionAndOrientation(
                    body, physicsClientId=cid)[0])
                probe += 1
                if not ok_all:
                    break
            if not ok_all:
                break
            masked = ovd.mask_samples(samples[s_mark:], col, k)
            res = recon_rays.reconstruct(masked, det_conf_min=a.det_conf_min)
            if not res:
                print(f"[창문 {k} {col}] 복원 실패 — 중단"); ok_all = False; break
            r0 = res[0]
            gtc = np.asarray(layout[k]["center"], float)
            err = float(np.linalg.norm(np.asarray(r0["center_w"], float) - gtc))
            print(f"[창문 {k} {col}] 복원 center=({r0['center_w'][0]:.2f},"
                  f"{r0['center_w'][1]:.2f},{r0['center_w'][2]:.2f})  "
                  f"GT오차 {err*1000:.0f} mm  시차각 {r0['min_parallax_deg']:.0f}deg  "
                  f"관측 {r0['n_obs']}")

            # 2) 그 창문 하나만 계획해서 통과
            wmap = [{"order_index": k, "color": col, "center": r0["center_w"],
                     "corners_3d": r0["corners_w"],
                     "size_wh": [r0["width"], r0["height"]]}]
            plv = planner.plan(wmap, start=tuple(cur), cfg=cfg)
            gates = [q for lb, q in zip(plv.labels, plv.waypoints)
                     if lb.startswith(("approach", "exit"))]
            # 탐색이 끝난 지점은 창문 법선에서 scan_y 만큼 옆으로 비켜 있다.
            # 거기서 곧장 접근 게이트로 들어가면 가로로 다 못 붙은 채 창문 평면에
            # 닿는다 (실측: 10창문에서 7개가 |u| 0.23~0.25 m 로 치우쳐 여유 미달).
            # 접근 게이트보다 더 뒤쪽, 법선 위에 정렬점을 하나 끼워 정면으로
            # 들어가게 만든다 — planner 가 창문 사이에 하는 것과 같은 처리다.
            # 다만 정렬점이 **직전 창문 뒤로** 가면 안 된다. 실측: #1 의 정렬점이
            # x=2.96 인데 드론은 #0 을 지나 x=3.84 에 있어서, 역주행으로 #0 을
            # 되돌아 통과하려다 창틀에 박았다. 그래서 법선 위 매개변수 t 를
            # "현재 위치보다 앞" 이 되도록 잘라 쓰고, 여유가 없으면 그냥 생략한다.
            cw = np.asarray(r0["center_w"], float)
            nrm = np.asarray(gates[0], float) - cw
            nrm = nrm / max(np.linalg.norm(nrm), 1e-9)        # 창문 -> 접근 방향
            t_want = float(cfg["d_app"]) + a.align_back
            if abs(nrm[0]) > 1e-6:
                t_lim = (cur[0] + 0.35 - cw[0]) / nrm[0]      # 현재 위치보다 앞
                t = min(t_want, t_lim) if nrm[0] < 0 else min(t_want, t_lim)
            else:
                t = t_want
            if t > float(cfg["d_app"]) + 0.15:
                algn = cw + t * nrm
                algn[2] = float(np.clip(algn[2], 0.6, 1.9))
                gates = [algn] + gates
            else:
                print(f"           (정렬점 생략 — 직전 창문과 너무 가깝다)")
            gates = np.asarray(gates, float)
            rlog.append({"k": k, "color": col, "err_mm": err * 1000.0,
                         "par": float(r0["min_parallax_deg"]), "n_obs": int(r0["n_obs"]),
                         "c_vis": np.asarray(r0["center_w"], float), "c_gt": gtc})
            s_mark = len(samples)      # 통과 구간부터가 다음 창문의 관측 구간
            legs.append(fly_leg(gates, spin=False))
            ok_all = legs[-1][1]
            print(f"           통과 비행 {'OK' if ok_all else '실패'}   "
                  f"누적 관측 {len(samples)}장")
            if not ok_all:
                break

        path = np.vstack([q for q, _ in legs])
        rows = passage_report(path, layout, margin)
        print("\n[통과 판정]")
        nok = 0
        for i_, r in enumerate(rows):
            if not r["passed"]:
                print(f"  #{i_} {r['color']:6s}  평면 통과 안 함"); continue
            nok += int(r["ok"])
            print(f"  #{i_} {r['color']:6s}  u={r['u']:+.3f} v={r['v']:+.3f} m   "
                  f"여유 {min(r['slack_u'],r['slack_v'])*1000:6.1f} mm   "
                  f"{'통과' if r['ok'] else '여유 미달'}")
        print(f"  -> {nok}/{len(rows)} 창문 안전 통과  (비전 계획, GT 미사용)")
        if rlog:
            e_ = np.array([r["err_mm"] for r in rlog])
            print(f"\n[비전 복원] {len(rlog)}/{len(layout)} 창문   GT오차 "
                  f"중앙값 {np.median(e_):.0f} mm  최대 {e_.max():.0f} mm   "
                  f"시차각 {min(r['par'] for r in rlog):.0f}~"
                  f"{max(r['par'] for r in rlog):.0f} deg")
        env.close()

        if a.gif:
            _write_pair(gif_3p, gif_fpv, a)

        # --- 그림 (라벨은 영어) --------------------------------------------
        viz.use_agg()
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(14, 9.5),
                               gridspec_kw={"height_ratios": [1, 1, 0.8]})
        for i, (iy, nm) in enumerate(((1, "y [m]"), (2, "z [m]"))):
            ax[i].plot(path[:, 0], path[:, iy], lw=1.6, color="#06c",
                       label="flown (vision-only planning)")
            for w in layout:
                c = np.asarray(w["center"], float)
                half = (w["ow"] if iy == 1 else w["oh"]) / 2
                ax[i].plot([c[0]] * 2, [c[iy] - half, c[iy] + half], lw=7,
                           color=viz.COL[w["color"]], solid_capstyle="butt", zorder=1)
                ax[i].plot([c[0]] * 2,
                           [c[iy] - half + margin, c[iy] + half - margin],
                           lw=2.5, color="w", solid_capstyle="butt", zorder=2)
            for r in rlog:
                ax[i].plot(r["c_gt"][0], r["c_gt"][iy], "o", ms=7, mfc="none",
                           mec="k", mew=1.4, zorder=4)
                ax[i].plot(r["c_vis"][0], r["c_vis"][iy], "x", ms=8, mew=2.0,
                           color="#d81b60", zorder=5)
            ax[i].set_ylabel(nm); ax[i].grid(alpha=.3)
        ax[0].plot([], [], "o", ms=7, mfc="none", mec="k", mew=1.4,
                   label="window centre, ground truth")
        ax[0].plot([], [], "x", ms=8, mew=2.0, color="#d81b60",
                   label="window centre, reconstructed from vision")
        ax[0].legend(fontsize=8, ncol=3, loc="upper left")
        ax[0].set_title("thick bar = opening,  white core = opening minus "
                        f"{margin*1000:.0f} mm clearance", fontsize=10)
        ax[1].set_xlabel("x [m]")
        if rlog:
            xs_ = [r["k"] for r in rlog]
            ax[2].bar(xs_, [r["err_mm"] for r in rlog],
                      color=[viz.COL[r["color"]] for r in rlog], width=0.6)
            ax[2].set_xticks(range(len(layout)))
            ax[2].set_ylabel("reconstruction error\nvs ground truth [mm]")
            ax[2].set_xlabel("window index (traversal order)")
            ax[2].grid(alpha=.3, axis="y")
            for r in rlog:
                ax[2].text(r["k"], r["err_mm"], f"{r['par']:.0f}\u00b0",
                           ha="center", va="bottom", fontsize=7.5, color="0.35")
            ax[2].set_title("bar = centre error,  label = parallax angle achieved",
                            fontsize=9.5)
        fig.suptitle(f"Vision-only traversal, {len(layout)} windows — "
                     f"no ground-truth planning.  "
                     f"{nok}/{len(rows)} passed with full clearance.",
                     fontsize=12, fontweight="bold")
        fig.tight_layout(rect=[0, 0, 1, 0.96])
        os.makedirs(os.path.dirname(a.fig) or ".", exist_ok=True)
        fig.savefig(a.fig, dpi=140)
        print(f"wrote {a.fig}")
        return 0 if nok == len(rows) else 1

    if a.replan:
        print("\n=== 재계획 모드: GT 창문을 계획에 쓰지 않는다 ===")
        print(f"    신뢰 문턱: 시차각 >= {a.trust_parallax} deg\n")
        p0 = np.array(p.getBasePositionAndOrientation(body, physicsClientId=cid)[0])
        legs = [fly_leg(p0 + np.array([a.lead_m, 0.0, 0.0]), spin=True)]
        print(f"[구간 1] 복도 전진 {a.lead_m} m  -> 관측 {len(samples)}장  "
              f"{'OK' if legs[-1][1] else '실패'}")
        done = set()          # 이미 통과한 창문
        ok_all = legs[-1][1]

        # 창문을 하나씩 밀고 나간다. 매 구간이 다음 창문의 관측 구간이 된다.
        for leg in range(2, a.max_legs + 1):
            if not ok_all:
                break
            res = recon_rays.reconstruct(samples, det_conf_min=a.det_conf_min)
            cur = np.array(p.getBasePositionAndOrientation(body,
                                                           physicsClientId=cid)[0])
            trust = [r for r in res
                     if r["min_parallax_deg"] >= a.trust_parallax
                     and r["order_index"] not in done
                     and r["center_w"][0] > cur[0] + 0.2]
            trust.sort(key=lambda r: r["center_w"][0])
            par = "  ".join(f"{r['color']}:{r['min_parallax_deg']:.0f}deg" for r in res)
            print(f"[구간 {leg}] 복원 시차각  {par}")
            if not trust:
                # 믿을 창문이 없으면 **더 전진해서 시차를 번다**. 여기서 멈추면
                # 교착이다 — 시차는 움직여야만 쌓이는데 안 움직이니 영원히 못 믿는다.
                nxt = [r for r in res if r["order_index"] not in done
                       and r["center_w"][0] > cur[0] + 0.2]
                stop_x = (min(r["center_w"][0] for r in nxt) - cfg["d_app"]
                          if nxt else cur[0] + a.lead_m)
                step = float(np.clip(stop_x - cur[0], 0.4, a.lead_m))
                tgt = cur + np.array([step, 0.0, 0.0])
                tgt[2] = float(np.clip(tgt[2], 0.6, 1.9))
                print(f"         신뢰할 창문 없음 -> 탐색 전진 {step:.2f} m")
                legs.append(fly_leg(tgt, spin=False))
                ok_all = legs[-1][1]
                print(f"         비행 {'OK' if ok_all else '실패'}   "
                      f"누적 관측 {len(samples)}장")
                continue
            wmap = [{"order_index": r["order_index"], "color": r["color"],
                     "center": r["center_w"], "corners_3d": r["corners_w"],
                     "size_wh": [r["width"], r["height"]]} for r in trust]
            plv = planner.plan(wmap, start=tuple(cur), cfg=cfg)
            names = ",".join(r["color"] for r in trust)
            print(f"         계획 대상 [{names}]  {planner.describe(plv)}")
            legs.append(fly_leg(np.array(plv.waypoints)[1:], spin=False))
            ok_all = legs[-1][1]
            done.update(r["order_index"] for r in trust)
            print(f"         비행 {'OK' if ok_all else '실패'}   "
                  f"누적 관측 {len(samples)}장")

        path = np.vstack([q for q, _ in legs])
        ok1 = legs[0][1]; ok2 = ok_all
        rows = passage_report(path, layout, margin)
        print("\n[통과 판정]")
        nok = 0
        for r in rows:
            if not r["passed"]:
                print(f"  {r['color']:6s}  평면 통과 안 함"); continue
            nok += int(r["ok"])
            print(f"  {r['color']:6s}  u={r['u']:+.3f} v={r['v']:+.3f} m   "
                  f"여유 {min(r['slack_u'],r['slack_v'])*1000:6.1f} mm   "
                  f"{'통과' if r['ok'] else '여유 미달'}")
        print(f"  -> {nok}/{len(rows)} 창문 안전 통과   (1구간 {'OK' if ok1 else 'X'}, "
              f"2구간 {'OK' if ok2 else 'X'})")
        res2 = recon_rays.reconstruct(samples, det_conf_min=a.det_conf_min)
        print("\n[최종 복원]"); metrics.print_rows(metrics.score(res2, layout))
        env.close()
        return 0 if (nok == len(rows) and ok1 and ok2) else 1

    pos0, quat0 = p.getBasePositionAndOrientation(body, physicsClientId=cid)
    for _ in range(int(a.spin_up / a.dt)):
        ctl.step(ref[0], 0.0, pos0, (0, 0, 0), a.dt)
        p.resetBasePositionAndOrientation(body, pos0, quat0, physicsClientId=cid)
        p.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0], physicsClientId=cid)

    # --- 비행 ---------------------------------------------------------------
    n = len(t)
    path = np.zeros((n, 3))
    rpy_log = np.zeros((n, 3))
    contacts = 0
    for k in range(n):
        pos, quat = p.getBasePositionAndOrientation(body, physicsClientId=cid)
        rpy = p.getEulerFromQuaternion(quat)
        th, dq = ctl.step(ref[k], 0.0, pos, rpy, a.dt)      # pose 만 전달
        qc.apply_to_body(p, body, th, dq, ctl.motor_xy, ctl.mix_dir, cid)
        p.stepSimulation(physicsClientId=cid)
        path[k], rpy_log[k] = pos, rpy
        if det is not None and k % a.render_every == 0:
            samples.append(observe_now(int(k * a.dt * 1e9), k))
        if rec is not None:
            img_e = None
            if k % rec.cam_every == 0:
                R_WB_e = quat_xyzw_to_rot(np.asarray(quat, float))
                fwd_e = R_WB_e[:, 0]
                img_e = pbs.render_frame(p, cid, np.asarray(pos, float),
                                         np.asarray(pos, float) + fwd_e, intr,
                                         scale=a.render_scale)
            rec.step(p, body, cid, image=img_e)
        if a.gif and k % a.gif_every == 0:
            _f3, _f1 = shot_pair(pos, quat)
            gif_3p.append(_f3); gif_fpv.append(_f1)
        if p.getContactPoints(bodyA=body, physicsClientId=cid):
            contacts += 1
        if not np.all(np.isfinite(pos)) or abs(pos[2]) > 40:
            print(f"  발산 t={k*a.dt:.2f}s")
            path, rpy_log, ref, t = path[:k+1], rpy_log[:k+1], ref[:k+1], t[:k+1]
            break

    err = np.linalg.norm(path - ref, axis=1)
    print(f"\n[비행] {len(t)*a.dt:.2f} s   추종 오차 RMS {np.sqrt((err**2).mean())*1000:.1f} mm"
          f"   최대 {err.max()*1000:.1f} mm")
    print(f"       자세 |roll| {np.abs(np.degrees(rpy_log[:,0])).max():.1f}°  "
          f"|pitch| {np.abs(np.degrees(rpy_log[:,1])).max():.1f}°   "
          f"충돌 스텝 {contacts}")
    print(f"       종점 [{path[-1,0]:.3f}, {path[-1,1]:.3f}, {path[-1,2]:.3f}]  "
          f"(목표 [{ref[-1,0]:.3f}, {ref[-1,1]:.3f}, {ref[-1,2]:.3f}])")

    if det is not None:
        n_det = sum(1 for s_ in samples if s_["detection"])
        print(f"\n[관측] {len(samples)}장 촬영, 검출 {n_det}장 "
              f"({100*n_det/max(1,len(samples)):.0f}%)")
        # 창문이 3개를 넘으면 색(r,g,b)이 순환해서 order_index 가 겹친다.
        # 통과 순서로 재배정하지 않으면 창문 4개의 광선이 한 바구니에 섞인다
        # (실측: 10창문에서 red 오차 9671 mm, 7개는 복원 실패).
        smp = (ovd.assign_order_by_passing(samples, layout)
               if len(layout) > 3 else samples)
        res = recon_rays.reconstruct(smp, det_conf_min=a.det_conf_min)
        vrows = metrics.score(res, layout)
        print("[복원] 비행 중 관측만으로:")
        metrics.print_rows(vrows)
        # 비전으로 계획했다면 어땠을지 (실제 비행은 GT 계획 그대로였다)
        if res:
            wmap = [{"order_index": r["order_index"], "color": r["color"],
                     "center": r["center_w"], "corners_3d": r["corners_w"],
                     "size_wh": [r["width"], r["height"]]} for r in res]
            plv = planner.plan(wmap, start=(0.0, 0.0, 1.0), cfg=cfg)
            gtw = np.array([q for lb, q in zip(pl.labels, pl.waypoints)
                            if lb.startswith(("approach", "exit"))])
            vsw = np.array([q for lb, q in zip(plv.labels, plv.waypoints)
                            if lb.startswith(("approach", "exit"))])
            if len(gtw) == len(vsw):
                d = np.linalg.norm(gtw - vsw, axis=1) * 1000
                print(f"[계획 비교] 비전 계획 vs GT 계획 게이트 오차  "
                      f"중앙값 {np.median(d):.0f} mm  최대 {d.max():.0f} mm")

    if rec is not None:
        info = rec.write(a.export_euroc)
        print(f"\n[EuRoC] {info['root']}")
        print(f"        cam0 {info['cam']}장  imu0 {info['imu']}개  "
              f"GT {info['gt']}개  ({info['dur_s']:.1f} s)")

    if a.gif:
        _write_pair(gif_3p, gif_fpv, a)

    rows = passage_report(path, layout, margin)
    print("\n[통과 판정]  u,v = 개구부 중심 기준 통과점")
    n_ok = 0
    for w, r in zip(layout, rows):
        if not r["passed"]:
            print(f"  {r['color']:6s}  평면 통과 안 함")
            continue
        n_ok += int(r["ok"])
        print(f"  {r['color']:6s}  u={r['u']:+.3f} v={r['v']:+.3f} m   "
              f"여유 {min(r['slack_u'], r['slack_v'])*1000:6.1f} mm   "
              f"{'통과' if r['ok'] else '여유 미달'}")
    print(f"  -> {n_ok}/{len(layout)} 창문 안전 통과")

    env.close()

    # --- 그림 ---------------------------------------------------------------
    viz.use_agg()
    import matplotlib.pyplot as plt
    W = np.array(pl.waypoints)
    fig, ax = plt.subplots(3, 1, figsize=(13, 9))
    for i, (iy, nm) in enumerate(((1, "y [m]"), (2, "z [m]"))):
        ax[i].plot(ref[:, 0], ref[:, iy], "--", lw=1.3, color="#888", label="reference")
        ax[i].plot(path[:, 0], path[:, iy], lw=1.8, color="#06c", label="flown")
        ax[i].plot(W[:, 0], W[:, iy], "o", ms=5, color="#e60", label="waypoints")
        for w in layout:
            c = np.asarray(w["center"], float)
            half = (w["ow"] if iy == 1 else w["oh"]) / 2
            ax[i].plot([c[0]] * 2, [c[iy] - half, c[iy] + half], lw=6,
                       color=viz.COL[w["color"]], solid_capstyle="butt")
            ax[i].plot([c[0]] * 2, [c[iy] - half + margin, c[iy] + half - margin],
                       lw=2, color="w", solid_capstyle="butt")
        ax[i].set_ylabel(nm); ax[i].grid(alpha=.3)
    ax[0].legend(fontsize=8, ncol=3)
    ax[0].set_title("thick bar = opening, white core = opening minus 350 mm clearance",
                    fontsize=10)
    ax[1].set_xlabel("x [m]")
    ax[2].plot(t, err * 1000, lw=1.4, color="#c33")
    ax[2].set_xlabel("time [s]"); ax[2].set_ylabel("tracking error [mm]")
    ax[2].grid(alpha=.3)
    fig.suptitle(f"Plan -> trajectory -> Seoungjin's controller -> PyBullet   "
                 f"({n_ok}/{len(layout)} windows cleared, "
                 f"RMS {np.sqrt((err**2).mean())*1000:.0f} mm, contacts {contacts})",
                 fontsize=12)
    viz.save(fig, os.path.dirname(a.fig), os.path.basename(a.fig), dpi=115)
    return 0 if n_ok == len(layout) and contacts == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
