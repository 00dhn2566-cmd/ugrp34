#!/usr/bin/env python3
"""qc_phys 에서 Gazebo 월드(SDF)를 생성한다.

왜 손으로 안 쓰고 생성하나: 관성/질량/로터 좌표를 손으로 적으면 `qc_controller.hpp`
의 `qc_phys()` 와 어긋나는 순간 "게인만 스케일되고 플랜트가 다른" 미스매치가 된다
(HANDOFF_CPP_GAZEBO.md "플랜트 물성 일치"). 여기서 한 번에 뽑아 두면 물성이 바뀔 때
이 파일만 다시 돌리면 된다.

    python worlds/gen_worlds.py          # worlds/fx450_qc_{1kg,0kg}.sdf 생성

로터 좌표는 **믹서 표에서 유도**한다 (아래 mixer_geometry 참조). 임의로 45도 X 를
가정하고 적으면 부호가 반대인 채로 날게 된다.
"""
from __future__ import annotations

import math
import pathlib
import sys

# -- qc_controller.hpp qc_phys() 의 1:1 사본 ------------------------------
M_CHASSIS = 0.9650346
Z_CHASSIS = +0.0038181
I_CHASSIS = (1.488e-3, 1.538e-3, 2.399e-3)
R_ARM = 0.225 / math.sqrt(2.0)      # 로터 반경 (FX450 대각 450 mm)
Z_ROTOR = +0.02
DRONE_MASS = 1.2726
PKG_SIZE = (0.14, 0.14, 0.14)


def qc_phys(m_drone, m_pkg, pkg_sz=PKG_SIZE):
    m_rot = m_drone - M_CHASSIS
    z_pkg = -0.012 - pkg_sz[2] / 2.0
    m_tot = m_drone + m_pkg
    z_cg = (M_CHASSIS * Z_CHASSIS + m_rot * Z_ROTOR + m_pkg * z_pkg) / m_tot
    dch2 = (Z_CHASSIS - z_cg) ** 2
    drot2 = (Z_ROTOR - z_cg) ** 2
    dpkg2 = (z_pkg - z_cg) ** 2
    ix = (I_CHASSIS[0] + M_CHASSIS * dch2 + m_rot * R_ARM ** 2 + m_rot * drot2
          + m_pkg / 12.0 * (pkg_sz[1] ** 2 + pkg_sz[2] ** 2) + m_pkg * dpkg2)
    iy = (I_CHASSIS[1] + M_CHASSIS * dch2 + m_rot * R_ARM ** 2 + m_rot * drot2
          + m_pkg / 12.0 * (pkg_sz[0] ** 2 + pkg_sz[2] ** 2) + m_pkg * dpkg2)
    iz = (I_CHASSIS[2] + m_rot * 2 * R_ARM ** 2
          + m_pkg / 12.0 * (pkg_sz[0] ** 2 + pkg_sz[1] ** 2))
    return dict(Ixx=ix, Iyy=iy, Izz=iz, I_att=0.5 * (ix + iy), I_yaw=iz,
                m_tot=m_tot, z_cg=z_cg, z_pkg=z_pkg, m_rot=m_rot)


# -- 믹서 표 -> 로터 좌표 -------------------------------------------------
# 08-18 골든 트레이스 실측표 (SESSIONS_BOARD 튜닝/C++ 세션 08-18 저녁).
MIX_PITCH = (+1, +1, -1, -1)
MIX_ROLL = (-1, +1, -1, +1)
MIX_YAW = (-1, +1, +1, -1)
MIX_DIR = (+1, -1, -1, +1)      # 모터 2,3 내장 역회전 (실측 w 부호 음수)


def mixer_geometry(arm_xy):
    """믹서 부호에서 각 모터의 (x, y) 사분면을 유도한다.

    Gazebo 기본 좌표계는 FLU (x 앞 / y 왼쪽 / z 위) 이고 토크 tau = sum r_i x F_i,
    F_i = (0, 0, T_i) 이므로

        tau_x (roll)  = + sum y_i T_i
        tau_y (pitch) = - sum x_i T_i

    이다. 명령 u 가 양수일 때 mix_i = +1 인 모터의 추력이 커지므로,
      +pitch (= tau_y > 0) 는 x_i < 0 인 모터를 올려야 하고,
      +roll  (= tau_x > 0) 는 y_i > 0 인 모터를 올려야 한다.

    부호 사슬 확인 (FLU): +x 오차 -> cmdPitch > 0 -> tau_y > 0 -> 기수 하강 ->
    추력 벡터가 +x 로 기울어 +x 로 가속. roll 은 cmdRoll = -by 라 +y 오차 ->
    cmdRoll < 0 -> tau_x < 0 -> +y 가속. 둘 다 자기정합.
    """
    pos = []
    for i in range(4):
        x = -arm_xy if MIX_PITCH[i] > 0 else +arm_xy
        y = +arm_xy if MIX_ROLL[i] > 0 else -arm_xy
        pos.append((x, y))
    return pos


def check_tables():
    """믹서 표 자기검증. 실패하면 그 표로는 날 수 없다."""
    rows = {"pitch": MIX_PITCH, "roll": MIX_ROLL, "yaw": MIX_YAW}
    problems = []
    keys = list(rows)
    for i, a in enumerate(keys):
        for b in keys[i + 1:]:
            d = sum(p * q for p, q in zip(rows[a], rows[b]))
            if d != 0:
                problems.append("%s/%s 축이 직교하지 않음 (내적 %+d)" % (a, b, d))
        if sum(rows[a]) != 0:
            problems.append("%s 행이 추력 행과 결합됨 (합 %+d)" % (a, sum(rows[a])))
    # yaw 권한은 반토크 차동이라 회전방향 패턴과 정렬돼야 한다.
    # (C++ 헤더 표 mixYaw = -1,+1,-1,+1 은 mixDir 와 직교라 yaw 토크가 0 이다.)
    if tuple(-d for d in MIX_DIR) != MIX_YAW and tuple(MIX_DIR) != MIX_YAW:
        problems.append("mixYaw %s 가 +-mixDir %s 가 아님 -> yaw 권한 0"
                        % (list(MIX_YAW), list(MIX_DIR)))
    # 같은 대각선의 두 모터는 같은 방향으로 돌아야 한다 (X 쿼드 규약).
    pos = mixer_geometry(1.0)
    for i in range(4):
        for j in range(i + 1, 4):
            if pos[i][0] == -pos[j][0] and pos[i][1] == -pos[j][1]:
                if MIX_DIR[i] != MIX_DIR[j]:
                    problems.append("대각 모터 %d/%d 회전방향이 반대" % (i + 1, j + 1))
    return problems


ROTOR_COLOR = {+1: "0.85 0.2 0.2", -1: "0.2 0.35 0.9"}


def world_sdf(name, m_pkg, arm_xy, wind_effects=None):
    ph = qc_phys(DRONE_MASS, m_pkg, PKG_SIZE)
    pos = mixer_geometry(arm_xy)
    out = []
    add = out.append
    add('<?xml version="1.0" ?>')
    add("<!--")
    add("  %s - control_seoungjin/gazebo/worlds/gen_worlds.py 가 생성. 손으로 고치지 말 것." % name)
    add("")
    add("  플랜트는 강체 1개 (base_link) 다. 로터를 별도 링크/조인트로 두지 않는 이유:")
    add("    - 관성이 qc_phys() 합성값과 정확히 같아야 하는데, 로터를 링크로 두면")
    add("      Gazebo 가 관성을 다시 합산해 이중 계상된다.")
    add("    - Simulink 쪽도 로터 회전 관성을 자세 플랜트에 넣지 않는다 (추력/반토크만).")
    add("  추력/반토크는 qc_gz_controller 플러그인이 AddWorldWrench 로 직접 인가한다.")
    add("")
    add("  m_pkg = %.3f kg (용접 강체 - 진자 조인트 아님)" % m_pkg)
    add("  m_tot = %.6f kg   z_cg = %+.6f m (섀시 원점 기준)" % (ph["m_tot"], ph["z_cg"]))
    add("  I_att = %.6e   I_yaw = %.6e kg*m^2" % (ph["I_att"], ph["I_yaw"]))
    add("  로터 |x|=|y| = %.6f m  (믹서 부호표에서 유도 - gen_worlds.mixer_geometry)" % arm_xy)
    add("    모터  (   x   ,    y   )   회전")
    for i, (x, y) in enumerate(pos):
        add("      %d   (%+.4f, %+.4f)   %s" % (i + 1, x, y, "CCW" if MIX_DIR[i] > 0 else "CW"))
    add("-->")
    add('<sdf version="1.9">')
    add('  <world name="%s">' % name)
    add("")
    add("    <!-- 1 kHz 고정 스텝: 골든 트레이스 기준. 가변 스텝 금지 (PID 이산화가 후방차분). -->")
    add('    <physics name="1ms" type="ignored">')
    add("      <max_step_size>0.001</max_step_size>")
    add("      <real_time_factor>0</real_time_factor>")
    add("    </physics>")
    add("")
    add('    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>')
    add('    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>')
    add('    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>')
    add("")
    add("    <!-- Gazebo 자체 외란 주입 API. 시뮬이 도는 중에 밖에서 링크에 렌치를 찔러")
    add("         넣을 수 있다 (scripts/poke.sh). 플러그인이 스스로 거는 외란과 **합산**된다.")
    add("           gz topic -t /world/%s/wrench -m gz.msgs.EntityWrench ..." % name)
    add("         힘은 무게중심에 걸리므로, '어디에 맞았나'를 표현하려면 등가 토크를 같이")
    add("         줘야 한다. 작용점을 그대로 주고 싶으면 플러그인의 distPoint* 를 쓸 것. -->")
    add('    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>')
    if wind_effects:
        add("")
        add("    <!-- 속도 의존 바람 (Gazebo 기본 모델). 상수 힘이 아니라 대기속도에 따라")
        add("         힘이 달라진다. 무게중심에 걸리므로 모멘트는 안 생긴다 — 모멘트가")
        add("         필요하면 플러그인의 windX/Y/Z + distPoint* 쪽을 쓸 것. -->")
        add("    <wind>")
        add("      <linear_velocity>%.3f %.3f %.3f</linear_velocity>" % wind_effects)
        add("    </wind>")
        add('    <plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">')
        add("      <force_approximation_scaling_factor>1.0</force_approximation_scaling_factor>")
        add("    </plugin>")
    add("")
    add("    <gravity>0 0 -9.81</gravity>")
    add('    <light type="directional" name="sun">')
    add("      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>")
    add("      <diffuse>1 1 1 1</diffuse><specular>0.4 0.4 0.4 1</specular>")
    add("      <direction>-0.5 0.1 -0.9</direction>")
    add("    </light>")
    add("")
    add('    <model name="ground_plane">')
    add("      <static>true</static>")
    add('      <link name="link">')
    add('        <collision name="collision">')
    add("          <geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>")
    add("        </collision>")
    add('        <visual name="visual">')
    add("          <geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>")
    add("          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>")
    add("        </visual>")
    add("      </link>")
    add("    </model>")
    add("")
    add('    <model name="fx450">')
    add("      <pose>0 0 0.12 0 0 0</pose>")
    add('      <link name="base_link">')
    if wind_effects:
        add("        <enable_wind>true</enable_wind>")
    add("        <inertial>")
    add("          <pose>0 0 %.7f 0 0 0</pose>" % ph["z_cg"])
    add("          <mass>%.7f</mass>" % ph["m_tot"])
    add("          <inertia>")
    add("            <ixx>%.7e</ixx><iyy>%.7e</iyy><izz>%.7e</izz>" % (ph["Ixx"], ph["Iyy"], ph["Izz"]))
    add("            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>")
    add("          </inertia>")
    add("        </inertial>")
    add('        <collision name="body_col">')
    add("          <geometry><box><size>0.20 0.20 0.06</size></box></geometry>")
    add("        </collision>")
    add('        <visual name="body">')
    add("          <geometry><box><size>0.20 0.20 0.06</size></box></geometry>")
    add("          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.16 0.16 0.16 1</diffuse></material>")
    add("        </visual>")
    if m_pkg > 0:
        add('        <visual name="package">')
        add("          <pose>0 0 %.4f 0 0 0</pose>" % ph["z_pkg"])
        add("          <geometry><box><size>%.3f %.3f %.3f</size></box></geometry>" % PKG_SIZE)
        add("          <material><ambient>0.55 0.4 0.2 1</ambient><diffuse>0.75 0.55 0.3 1</diffuse></material>")
        add("        </visual>")
        add('        <collision name="package_col">')
        add("          <pose>0 0 %.4f 0 0 0</pose>" % ph["z_pkg"])
        add("          <geometry><box><size>%.3f %.3f %.3f</size></box></geometry>" % PKG_SIZE)
        add("        </collision>")
    for i, (x, y) in enumerate(pos):
        yaw = math.atan2(y, x)
        add('        <visual name="arm_%d">' % (i + 1))
        add("          <pose>%.4f %.4f %.4f 0 1.5708 %.4f</pose>" % (x / 2, y / 2, Z_ROTOR / 2, yaw))
        add("          <geometry><cylinder><radius>0.009</radius><length>%.4f</length></cylinder></geometry>"
            % math.hypot(x, y))
        add("          <material><diffuse>0.25 0.25 0.25 1</diffuse></material>")
        add("        </visual>")
        add('        <visual name="rotor_%d">' % (i + 1))
        add("          <pose>%.4f %.4f %.4f 0 0 0</pose>" % (x, y, Z_ROTOR))
        add("          <geometry><cylinder><radius>0.127</radius><length>0.006</length></cylinder></geometry>")
        add("          <material><ambient>%s 1</ambient><diffuse>%s 1</diffuse></material>"
            % (ROTOR_COLOR[MIX_DIR[i]], ROTOR_COLOR[MIX_DIR[i]]))
        add("        </visual>")
    add("      </link>")
    add("")
    add("      <!-- 제어기 + 플랜트 접합. 모든 값은 QC_* 환경변수로 덮어쓸 수 있다")
    add("           (scripts/run_case.sh 가 그렇게 쓴다 - 월드 파일은 안 건드림). -->")
    add('      <plugin filename="qc_gz_controller" name="qc::QcGzController">')
    add("        <link>base_link</link>")
    add("        <mode>hover</mode>")
    add("        <pkgMass>%.4f</pkgMass>" % m_pkg)
    add("        <profile>precision</profile>")
    add("        <armXY>%.6f</armXY>" % arm_xy)
    add("        <rotorZ>%.6f</rotorZ>" % Z_ROTOR)
    add("        <comZ>%.7f</comZ>" % ph["z_cg"])
    add("        <mixerTable>measured</mixerTable>")
    if m_pkg < 0.5:
        add("        <!-- 저질량은 질량 1차식이 없으면 뜨지 못한다: 무보정 biasChassis 56.5")
        add("             rev/s = 추력 6.98 N < 생 드론 무게 12.48 N. 1차식 0 kg 앵커 75.5 가")
        add("             필요한 75.6 과 맞는다. precision 프로파일에서만 안전 (프로파일 충돌). -->")
        add("        <massLerpOn>true</massLerpOn>")
    add("        <hoverZ>1.0</hoverZ>")
    add("        <controlRateHz>1000</controlRateHz>")
    add("        <logRateHz>200</logRateHz>")
    add("        <log>gz_run.csv</log>")
    add("      </plugin>")
    add("    </model>")
    add("")
    add("  </world>")
    add("</sdf>")
    return "\n".join(out) + "\n"


def main(argv=None):
    # --wind-effects "x y z": Gazebo 기본 바람 모델을 켠다 (기본 꺼짐).
    # 기본을 끄는 이유: 물리가 달라지는데 이 노트북에서 검증할 수 없다.
    argv = list(sys.argv[1:] if argv is None else argv)
    wind_effects = None
    if "--wind-effects" in argv:
        k = argv.index("--wind-effects")
        try:
            wind_effects = tuple(float(v) for v in argv[k + 1].split())
            if len(wind_effects) != 3:
                raise ValueError
        except (IndexError, ValueError):
            print('사용법: --wind-effects "vx vy vz"  (m/s)')
            return 2
    problems = check_tables()
    here = pathlib.Path(__file__).resolve().parent
    print("믹서 표 자기검증:", "통과" if not problems else "실패")
    for p in problems:
        print("   !", p)
    if problems:
        print("   -> 월드를 생성하지 않는다. 표를 고치기 전에는 폐루프 금지.")
        return 1
    arm_xy = R_ARM / math.sqrt(2.0)      # 45도 X 기하에서 로터의 x=y 성분
    for tag, m_pkg in (("1kg", 1.0), ("0kg", 0.0)):
        path = here / ("fx450_qc_%s.sdf" % tag)
        path.write_text(world_sdf("fx450_qc_%s" % tag, m_pkg, arm_xy, wind_effects),
                        encoding="utf-8")
        ph = qc_phys(DRONE_MASS, m_pkg)
        print("생성 %s: m_tot=%.4f kg  I_att=%.4e  I_yaw=%.4e  armXY=%.4f"
              % (path.name, ph["m_tot"], ph["I_att"], ph["I_yaw"], arm_xy))
    print("호버 로터속도 예측: %.1f rad/s (= 2*pi*(56.5 + 44.4*m_pkg))"
          % (2 * math.pi * (56.5 + 44.4 * 1.0)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
