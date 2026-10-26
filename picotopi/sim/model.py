# -*- coding: utf-8 -*-
"""1지(검지형) MJCF 생성기 (V8 hand_model.py와 동일).

V7은 저장소 밖의 universal_grasp_env / sts3215_sim2real_env에 의존했습니다. V8은 모델을 여기서 직접 만들어
저장소만으로 실행됩니다.

좌표: 손바닥 면 z=0, 손바닥은 -z를 향함. 손가락은 MCP(원점)에서 +x로 뻗고, 관절 +각도가 -z(물체 쪽) 굴곡.
구동: MCP = STS3215 위치 서보, PIP/DIP = 굴곡 텐던(MCP도 통과, ADR 005 연성) + PIP 복귀 스프링, DIP = 0.88·PIP.
물체: 원기둥. 반대편 지지(엄지·다른 손가락·손바닥)를 x/z 슬라이드 스프링으로 근사. 중력 0 (1지 모델에서 무게·미끄럼은
다지 버전에서 다룸).
"""
from .. import config as C


def build_xml(p):
    """p: 에피소드 파라미터 dict (env.reset에서 생성)."""
    r = p["radius"]
    w = C.LINK_W / 2
    xo, zo = -(r + w + 0.002), -(r + C.OBJ_GAP)
    k_opp = 800.0 * p["oppose"]
    tc = 0.004 * p["solref"]
    rs = p["r_scale"]
    fmax = p["tendon_fmax"] * p["eta"]
    kt = C.TENDON_KP * p["eta"]
    tau = C.STS_TAU_MAX * p["tau"]
    kp = C.STS_KP * p["kp"]
    return f"""
<mujoco model="v8_finger">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{C.SIM_DT}" gravity="0 0 0" integrator="implicitfast" cone="elliptic"/>
  <default>
    <geom condim="3" solref="{tc:.5f} 1" solimp="0.9 0.95 0.001" friction="{p['mu']:.3f} 0.005 0.0001" density="1200"/>
  </default>
  <worldbody>
    <body name="palm">
      <geom name="palm" type="box" size="0.035 0.012 0.004" pos="-0.035 0 0.004" contype="2" conaffinity="1"/>
      <body name="prox">
        <joint name="mcp" axis="0 1 0" range="0 {C.TH_MAX}" armature="{C.STS_ARMATURE * p['arm']:.6f}"
               damping="0.01" frictionloss="{C.STS_FRICTIONLOSS}"/>
        <geom name="prox" type="capsule" fromto="0 0 0 {C.L_PROX} 0 0" size="{w}" contype="2" conaffinity="1"/>
        <body name="mid" pos="{C.L_PROX} 0 0">
          <joint name="pip" axis="0 1 0" range="0 {C.PIP_MAX}" stiffness="{C.PIP_SPRING * p['spring']:.5f}"
                 springref="0" damping="0.002" armature="1e-5"/>
          <geom name="mid" type="capsule" fromto="0 0 0 {C.L_MID} 0 0" size="{w}" contype="2" conaffinity="1"/>
          <body name="dist" pos="{C.L_MID} 0 0">
            <joint name="dip" axis="0 1 0" range="0 1.40" damping="0.002" armature="1e-5"/>
            <geom name="dist" type="capsule" fromto="0 0 0 {C.L_DIST} 0 0" size="{w}" contype="2" conaffinity="1"/>
            <site name="tip" pos="{C.L_DIST} 0 0"/>
          </body>
        </body>
      </body>
    </body>
    <body name="obj" pos="{xo:.5f} 0 {zo:.5f}">
      <joint name="ox" type="slide" axis="1 0 0" stiffness="{k_opp:.2f}" damping="4" range="-0.01 0.01"/>
      <joint name="oz" type="slide" axis="0 0 1" stiffness="{k_opp:.2f}" damping="4" range="-0.01 0.01"/>
      <geom name="obj" type="cylinder" size="{r:.5f} 0.012" euler="1.5708 0 0" mass="{p['mass']:.4f}"
            contype="1" conaffinity="2"/>
    </body>
  </worldbody>
  <equality>
    <joint joint1="dip" joint2="pip" polycoef="0 {C.DIP_PIP_RATIO} 0 0 0" solref="0.004 1"/>
  </equality>
  <tendon>
    <fixed name="flexor">
      <joint joint="mcp" coef="{-C.R_MCP * rs:.6f}"/>
      <joint joint="pip" coef="{-C.R_PIP * rs:.6f}"/>
      <joint joint="dip" coef="{-C.R_DIP * rs:.6f}"/>
    </fixed>
  </tendon>
  <actuator>
    <position name="mcp_servo" joint="mcp" kp="{kp:.3f}" kv="0.05" forcerange="{-tau:.3f} {tau:.3f}" ctrlrange="0 {C.TH_MAX}"/>
    <position name="tendon_servo" tendon="flexor" kp="{kt:.2f}" kv="{0.02 * kt:.3f}" forcerange="{-fmax:.2f} 0" ctrlrange="-0.06 0.01"/>
  </actuator>
</mujoco>
"""
