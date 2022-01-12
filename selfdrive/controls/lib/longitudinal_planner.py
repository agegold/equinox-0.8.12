#!/usr/bin/env python3
import math
import numpy as np
from common.numpy_fast import interp

import cereal.messaging as messaging
from common.realtime import DT_MDL
from selfdrive.modeld.constants import T_IDXS
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.longcontrol import LongCtrlState
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N
from selfdrive.swaglog import cloudlog
from selfdrive.ntune import ntune_scc_get

LON_MPC_STEP = 0.2  # first step is 0.2s
AWARENESS_DECEL = -0.2  # car smoothly decel at .2m/s^2 when user is distracted
A_CRUISE_MIN = -6.0
A_CRUISE_MAX_VALS = [1.5, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 15., 25., 40.]

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

DP_ACCEL_ECO = 1
DP_ACCEL_NORMAL = 2
DP_ACCEL_SPORT = 3

# accel profile by @arne182 modified by @wer5lcy
#_DP_CRUISE_MIN_V_ECO = [-2.0, -1.6, -1.4, -1.2, -1.0]
#_DP_CRUISE_MIN_V_NORMAL = [-2.0, -1.8, -1.6, -1.4, -1.2]
#_DP_CRUISE_MIN_V_SPORT = [-3.0, -2.0, -1.8, -1.6, -1.4]
#_DP_CRUISE_MIN_BP = [0.0, 5.0, 10.0, 20.0, 30.0]

_DP_CRUISE_MIN_V_ECO = [-6.0, -6.0, -4.0, -3.5, -3.0]
_DP_CRUISE_MIN_V_NORMAL = [-6.5, -6.5, -5.0, -5.0, -4.0]
_DP_CRUISE_MIN_V_SPORT = [-7.0, -7.0, -6.0, -5.0, -4.5]
_DP_CRUISE_MIN_BP = [0.0, 5.0, 10.0, 20.0, 30.0]

_DP_CRUISE_MAX_V_ECO = [1.5, 1.3, 0.8, 0.6, 0.4]
_DP_CRUISE_MAX_V_NORMAL = [1.6, 1.4, 1.0, 0.8, 0.6]
_DP_CRUISE_MAX_V_SPORT = [1.7, 1.5, 1.1, 1.0, 0.8]
_DP_CRUISE_MAX_BP = [0., 5., 10., 20., 30.]

def dp_calc_cruise_accel_limits(v_ego, dp_profile):
  if dp_profile == DP_ACCEL_ECO:
    a_cruise_min = interp(v_ego, _DP_CRUISE_MIN_BP, _DP_CRUISE_MIN_V_ECO)
    a_cruise_max = interp(v_ego, _DP_CRUISE_MAX_BP, _DP_CRUISE_MAX_V_ECO)
  elif dp_profile == DP_ACCEL_SPORT:
    a_cruise_min = interp(v_ego, _DP_CRUISE_MIN_BP, _DP_CRUISE_MIN_V_SPORT)
    a_cruise_max = interp(v_ego, _DP_CRUISE_MAX_BP, _DP_CRUISE_MAX_V_SPORT)
  else:
    a_cruise_min = interp(v_ego, _DP_CRUISE_MIN_BP, _DP_CRUISE_MIN_V_NORMAL)
    a_cruise_max = interp(v_ego, _DP_CRUISE_MAX_BP, _DP_CRUISE_MAX_V_NORMAL)
  return a_cruise_min, a_cruise_max


def get_max_accel(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class Planner:
  def __init__(self, CP, init_v=0.0, init_a=0.0):
    self.CP = CP
    self.mpc = LongitudinalMpc()

    self.fcw = False

    self.v_desired = init_v
    self.a_desired = init_a
    self.alpha = np.exp(-DT_MDL / 2.0)

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm):
    v_ego = sm['carState'].vEgo
    a_ego = sm['carState'].aEgo

    v_cruise_kph = sm['controlsState'].vCruise
    v_cruise_kph = min(v_cruise_kph, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    # neokii
    #if not self.use_cluster_speed:
    vCluRatio = sm['carState'].vCluRatio
    if vCluRatio > 0.5:
      v_cruise *= vCluRatio
      v_cruise = int(v_cruise * CV.MS_TO_KPH) * CV.KPH_TO_MS

    long_control_state = sm['controlsState'].longControlState
    force_slow_decel = sm['controlsState'].forceDecel

    prev_accel_constraint = True
    if long_control_state == LongCtrlState.off or sm['carState'].gasPressed:
      self.v_desired = v_ego
      self.a_desired = 0.0 #a_ego
      # Smoothly changing between accel trajectory is only relevant when OP is driving
      prev_accel_constraint = False

    # Prevent divergence, smooth in current v_ego
    self.v_desired = self.alpha * self.v_desired + (1 - self.alpha) * v_ego
    self.v_desired = max(0.0, self.v_desired)

    # PSK .....
    accelProfile = ntune_scc_get('accelProfile')
    if accelProfile == 0:
      accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
    else:
      accel_limits = dp_calc_cruise_accel_limits(v_ego, accelProfile)

    accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)
    if force_slow_decel:
      # if required so, force a smooth deceleration
      accel_limits_turns[1] = min(accel_limits_turns[1], AWARENESS_DECEL)
      accel_limits_turns[0] = min(accel_limits_turns[0], accel_limits_turns[1])
    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)
    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired, self.a_desired)
    self.mpc.update(sm['carState'], sm['radarState'], v_cruise, prev_accel_constraint=prev_accel_constraint)
    self.v_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 5
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(interp(DT_MDL, T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired = self.v_desired + DT_MDL * (self.a_desired + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_alive_and_valid(service_list=['carState', 'controlsState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']

    longitudinalPlan.speeds = [float(x) for x in self.v_desired_trajectory]
    longitudinalPlan.accels = [float(x) for x in self.a_desired_trajectory]
    longitudinalPlan.jerks = [float(x) for x in self.j_desired_trajectory]

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    pm.send('longitudinalPlan', plan_send)