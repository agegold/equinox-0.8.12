from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import interp, clip
from selfdrive.config import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits, create_gas_interceptor_command
from selfdrive.car.gm import gmcan
from selfdrive.car.gm.values import DBC, CanBus, CarControllerParams
from opendbc.can.packer import CANPacker
from selfdrive.ntune import ntune_scc_get

VisualAlert = car.CarControl.HUDControl.VisualAlert

VEL = [13.889, 16.667, 25.]  # velocities
MIN_PEDAL = [0.02, 0.05, 0.1]


def accel_hysteresis(accel, accel_steady):
    # for small accel oscillations less than 0.02, don't change the accel command
    if accel > accel_steady + 0.02:
        accel_steady = accel - 0.02
    elif accel < accel_steady - 0.02:
        accel_steady = accel + 0.02
    accel = accel_steady

    return accel, accel_steady


def compute_gas_brake(accel, speed):
  creep_brake = 0.0
  creep_speed = 2.3
  creep_brake_value = 0.15
  if speed < creep_speed:
    creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value
  gb = float(accel) / 4.8 - creep_brake
  return clip(gb, 0.0, 1.0), clip(-gb, 0.0, 1.0)


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.start_time = 0.
    self.apply_steer_last = 0
    self.lka_steering_cmd_counter_last = -1
    self.lka_icon_status_last = (False, False)
    self.steer_rate_limited = False
    
    self.accel_steady = 0.
    #self.apply_brake = 0
    
    self.params = CarControllerParams()

    self.packer_pt = CANPacker(DBC[CP.carFingerprint]['pt'])
    #self.packer_obj = CANPacker(DBC[CP.carFingerprint]['radar'])
    #self.packer_ch = CANPacker(DBC[CP.carFingerprint]['chassis'])

  def update(self, enabled, CS, frame, actuators,
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert):

    P = self.params

    if enabled:
      accel = actuators.accel
      gas, brake = compute_gas_brake(actuators.accel, CS.out.vEgo)
    else:
      accel = 0.0
      gas, brake = 0.0, 0.0

    # Send CAN commands.
    can_sends = []

    # STEER

    # Steering (50Hz)
    # Avoid GM EPS faults when transmitting messages too close together: skip this transmit if we just received the
    # next Panda loopback confirmation in the current CS frame.
    if CS.lka_steering_cmd_counter != self.lka_steering_cmd_counter_last:
      self.lka_steering_cmd_counter_last = CS.lka_steering_cmd_counter
    elif (frame % P.STEER_STEP) == 0:
      lkas_enabled = enabled and not (CS.out.steerWarning or CS.out.steerError) and CS.out.vEgo > P.MIN_STEER_SPEED
      if lkas_enabled:
        new_steer = int(round(actuators.steer * P.STEER_MAX))
        apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
        self.steer_rate_limited = new_steer != apply_steer
      else:
        apply_steer = 0

      self.apply_steer_last = apply_steer
      # GM EPS faults on any gap in received message counters. To handle transient OP/Panda safety sync issues at the
      # moment of disengaging, increment the counter based on the last message known to pass Panda safety checks.
      idx = (CS.lka_steering_cmd_counter + 1) % 4

      can_sends.append(gmcan.create_steering_control(self.packer_pt, CanBus.POWERTRAIN, apply_steer, idx, lkas_enabled))

    if CS.CP.enableGasInterceptor:

      if not enabled or not CS.adaptive_Cruise or CS.out.vEgo <= 1 / CV.MS_TO_KPH:
        comma_pedal = 0.
        #apply_brake = 0
      elif CS.adaptive_Cruise and CS.out.vEgo > 1 / CV.MS_TO_KPH:
        # ????????? ????????? ???????????? ?????? ??????????????????.
        gas_mult = interp(CS.out.vEgo, [0., 10.], [0.4, 1.0])
        # apply_gas??? 0?????? ????????? 0??? ????????????. ??????????????? ?????? ?????? apply_gas ????????? ???????????? ????????????.
        # ????????? ?????? ?????? ?????? ???????????? ???????????????.
        # OP ??? ??????????????? ????????? ??? 0??? ?????? ????????? ????????? PCM??? ???????????? ???????????? ???????????? ????????????.
        # ???????????? ???.
        comma_pedal = clip(gas_mult * (gas - brake), 0., 1.)
        #apply_brake = int(round(interp(actuators.accel, P.BRAKE_LOOKUP_BP, P.BRAKE_LOOKUP_V)))

      if (frame % 4) == 0:
        idx = (frame // 4) % 4

        #at_full_stop = enabled and CS.out.standstill
        #near_stop = enabled and (CS.out.vEgo < P.NEAR_STOP_BRAKE_PHASE)
        #print("apply_brake : " , apply_brake)
        #print("near_stop : " , near_stop)
        #print("at_full_stop : ", + at_full_stop)
        #can_sends.append(gmcan.create_friction_brake_command(self.packer_ch, CanBus.CHASSIS, apply_brake, idx, near_stop,at_full_stop))
        can_sends.append(create_gas_interceptor_command(self.packer_pt, comma_pedal, idx))

    # Show green icon when LKA torque is applied, and
    # alarming orange icon when approaching torque limit.
    # If not sent again, LKA icon disappears in about 5 seconds.
    # Conveniently, sending camera message periodically also works as a keepalive.
    lka_active = CS.lkas_status == 1
    lka_critical = lka_active and abs(actuators.steer) > 0.9
    lka_icon_status = (lka_active, lka_critical)
    if frame % P.CAMERA_KEEPALIVE_STEP == 0 or lka_icon_status != self.lka_icon_status_last:
      steer_alert = hud_alert in [VisualAlert.steerRequired, VisualAlert.ldw]
      can_sends.append(gmcan.create_lka_icon_command(CanBus.SW_GMLAN, lka_active, lka_critical, steer_alert))
      self.lka_icon_status_last = lka_icon_status

    return can_sends
