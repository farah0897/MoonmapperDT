#!/usr/bin/env python3
"""Rocker-bogie + differential kinematic coupling for MoonMapper.

Driver-joints (fra JSP/GUI paa input_topic):
    L = rocker_left_joint
    R = rocker_right_joint

Avhengige joints beregnes affint (alle gains er ROS-parametere):

    q_diff    = k_diff_L * L   +  k_diff_R * R   +  k_diff_0
    q_hL      = d_L      * L                      +  e_L
    q_hR      = d_R      * R                      +  e_R
    q_bogieL  = a_bl     * L   +  b_bl * q_diff   +  c_bl
    q_bogieR  = a_br     * R   +  b_br * q_diff   +  c_br

Noden abonnerer på /joint_states_raw (det JSP-GUI publishes etter
remap), overskriver/tilføyer de avhengige jointene, og publishes
den komplette JointState til /joint_states. robot_state_publisher
bruker den som vanlig.
"""

from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RockerBogieKinematics(Node):
    def __init__(self) -> None:
        super().__init__('rocker_bogie_kinematics')

        # -------- joint-navn (overstyrbare via launch) ------------------
        # NB: Default-verdiene her skal matche URDF/Xacro i moonmapper_description.
        self.declare_parameter('left_rocker',  'rocker_left_joint')
        self.declare_parameter('right_rocker', 'rocker_right_joint')
        self.declare_parameter('left_bogie',   'bogie_left_joint')
        self.declare_parameter('right_bogie',  'bogie_right_joint')
        # Hengsel-joints finnes ikke i dagens moonmapper_rover.urdf.xacro.
        # La dem være tomme som default; hvis du faktisk har dem i URDF, sett navn via launch.
        self.declare_parameter('left_hinge',   '')
        self.declare_parameter('right_hinge',  '')
        self.declare_parameter('diff_joint',   'rocker_bogie_diff_joint')

        # -------- topics ------------------------------------------------
        self.declare_parameter('input_topic',  '/joint_states_raw')
        self.declare_parameter('output_topic', '/joint_states')

        # -------- diff: k_L*L + k_R*R + k0 ------------------------------
        # Default (0.5, 0.5, 0.0)  ->  q_diff = (L + R) / 2
        self.declare_parameter('k_diff_L', 0.5)
        self.declare_parameter('k_diff_R', 0.5)
        self.declare_parameter('k_diff_0', 0.0)

        # -------- hinges: d*rocker + e ----------------------------------
        self.declare_parameter('d_L', 1.0)
        self.declare_parameter('e_L', 0.0)
        self.declare_parameter('d_R', 1.0)
        self.declare_parameter('e_R', 0.0)

        # -------- bogies: a*rocker + b*diff + c -------------------------
        self.declare_parameter('a_bl', -0.5)
        self.declare_parameter('b_bl',  0.0)
        self.declare_parameter('c_bl',  0.0)
        self.declare_parameter('a_br', -0.5)
        self.declare_parameter('b_br',  0.0)
        self.declare_parameter('c_br',  0.0)

        self.declare_parameter('warn_timeout_sec', 5.0)

        g = self.get_parameter  # shorthand
        s = lambda n: g(n).get_parameter_value().string_value
        d = lambda n: g(n).get_parameter_value().double_value

        # Hent navnene én gang
        self.n_Lr, self.n_Rr = s('left_rocker'), s('right_rocker')
        self.n_Lb, self.n_Rb = s('left_bogie'),  s('right_bogie')
        self.n_Lh, self.n_Rh = s('left_hinge'),  s('right_hinge')
        self.n_d             = s('diff_joint')

        in_topic, out_topic = s('input_topic'), s('output_topic')

        # Hent gains
        self.k_dL, self.k_dR, self.k_d0 = d('k_diff_L'), d('k_diff_R'), d('k_diff_0')
        self.d_L,  self.e_L             = d('d_L'),  d('e_L')
        self.d_R,  self.e_R             = d('d_R'),  d('e_R')
        self.a_bl, self.b_bl, self.c_bl = d('a_bl'), d('b_bl'), d('c_bl')
        self.a_br, self.b_br, self.c_br = d('a_br'), d('b_br'), d('c_br')
        self.warn_timeout               = d('warn_timeout_sec')

        # Cachet driver-posisjon. None inntil vi har mottatt verdier.
        self._L: Optional[float] = None
        self._R: Optional[float] = None
        self._start = self.get_clock().now()
        self._warned = False

        self._sub = self.create_subscription(JointState, in_topic, self._on_joints, 10)
        self._pub = self.create_publisher(JointState, out_topic, 10)

        self.get_logger().info(
            'rocker_bogie_kinematics running\n'
            f'  input:  {in_topic}\n'
            f'  output: {out_topic}\n'
            f'  drivers: L={self.n_Lr}, R={self.n_Rr}\n'
            f'  q_diff    = {self.k_dL:+.3f}*L {self.k_dR:+.3f}*R {self.k_d0:+.3f}\n'
            f'  q_bogieL  = {self.a_bl:+.3f}*L {self.b_bl:+.3f}*diff {self.c_bl:+.3f}\n'
            f'  q_bogieR  = {self.a_br:+.3f}*R {self.b_br:+.3f}*diff {self.c_br:+.3f}\n'
            f'  q_hingeL  = {self.d_L:+.3f}*L {self.e_L:+.3f}\n'
            f'  q_hingeR  = {self.d_R:+.3f}*R {self.e_R:+.3f}'
        )

    # ------------------------------------------------------------------
    def _on_joints(self, msg: JointState) -> None:
        # Bygg navn->indeks-oppslag. Gjoeres pr. melding fordi
        # joint_state_publisher kan endre rekkefoelgen.
        idx = {n: i for i, n in enumerate(msg.name)}

        # Oppdater cachet driver-posisjon kun hvis joint eksisterer i input
        # og har en gyldig posisjon-indeks.
        if self.n_Lr in idx and idx[self.n_Lr] < len(msg.position):
            self._L = msg.position[idx[self.n_Lr]]
        if self.n_Rr in idx and idx[self.n_Rr] < len(msg.position):
            self._R = msg.position[idx[self.n_Rr]]

        # Vent paa begge driver-joints foer vi regner noe.
        if self._L is None or self._R is None:
            self._maybe_warn_missing()
            return

        L, R = self._L, self._R
        q_diff = self.k_dL * L + self.k_dR * R + self.k_d0
        q_hL   = self.d_L  * L + self.e_L
        q_hR   = self.d_R  * R + self.e_R
        q_bL   = self.a_bl * L + self.b_bl * q_diff + self.c_bl
        q_bR   = self.a_br * R + self.b_br * q_diff + self.c_br

        overrides = {
            self.n_d:  q_diff,
            self.n_Lb: q_bL,
            self.n_Rb: q_bR,
        }
        # Hengsel-joints er valgfrie (tom streng => ignorer)
        if self.n_Lh:
            overrides[self.n_Lh] = q_hL
        if self.n_Rh:
            overrides[self.n_Rh] = q_hR

        out = self._build_output(msg, idx, overrides)
        self._pub.publish(out)

    # ------------------------------------------------------------------
    def _build_output(
        self,
        msg: JointState,
        idx: dict,
        overrides: dict,
    ) -> JointState:
        """Kopier input, overskriv/appendér avhengige joints."""
        out = JointState()

        # Behold innkommende tidsstempel naar det finnes, slik at TF ser
        # én konsistent klokke; fall tilbake til nodens klokke ellers.
        has_stamp = bool(msg.header.stamp.sec or msg.header.stamp.nanosec)
        out.header.stamp = msg.header.stamp if has_stamp else self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id

        out.name     = list(msg.name)
        out.position = list(msg.position)

        has_vel = len(msg.velocity) == len(msg.name)
        has_eff = len(msg.effort)   == len(msg.name)
        if has_vel:
            out.velocity = list(msg.velocity)
        if has_eff:
            out.effort = list(msg.effort)

        for name, value in overrides.items():
            if name in idx and idx[name] < len(out.position):
                # Overskriv eksisterende slider-verdi saa GUI ikke overstyrer.
                i = idx[name]
                out.position[i] = value
                if has_vel:
                    out.velocity[i] = 0.0
                if has_eff:
                    out.effort[i] = 0.0
            else:
                # Appendér hvis jointen manglet helt i input.
                out.name.append(name)
                out.position.append(value)
                if has_vel:
                    out.velocity.append(0.0)
                if has_eff:
                    out.effort.append(0.0)

        return out

    # ------------------------------------------------------------------
    def _maybe_warn_missing(self) -> None:
        if self._warned:
            return
        elapsed = (self.get_clock().now() - self._start).nanoseconds * 1e-9
        if elapsed <= self.warn_timeout:
            return
        missing = []
        if self._L is None:
            missing.append(self.n_Lr)
        if self._R is None:
            missing.append(self.n_Rr)
        self.get_logger().warn(
            f'Mangler driver-joints etter {elapsed:.1f}s: {missing}. '
            f'Sjekk at JSP/GUI publishes dem paa input-topicen, '
            f'og at navnene stemmer med URDF.'
        )
        self._warned = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RockerBogieKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
