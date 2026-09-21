#!/usr/bin/env python3
"""Generate the student skeletons from the reference solutions.

Skeletons are *derived*, never maintained by hand. Two copies of the same file
with different bodies is how a lab ends up with a skeleton that cannot become
the solution it is graded against -- and nobody notices until the class.

Each entry below names a block of the solution and the hint that replaces it.
The match must be exact and unique, or this script fails loudly.

    python3 exercises/make_skeletons.py
"""
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SOL = os.path.join(HERE, "solutions")
SKEL = os.path.join(HERE, "skeletons")


def todo(hint, indent="        ", extra=()):
    lines = [f"{indent}# TODO(student): {hint}"]
    lines += [f"{indent}# {e}" for e in extra]
    return "\n".join(lines)


# (package, file, [(exact block, replacement)])
CUTS = {
    ("lab3_offboard", "scripts/offboard_square.py"): [
        ("""        m = PositionTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        m.type_mask = POSITION_ONLY
        m.position.x, m.position.y, m.position.z = x, y, z
        m.yaw = yaw
        self.sp.publish(m)""",
         todo("fill in a PositionTarget and publish it on self.sp.",
              extra=["coordinate_frame must be PositionTarget.FRAME_LOCAL_NED",
                     "(despite the name, MAVROS has already converted: x East,",
                     "y North, z Up), and type_mask must be POSITION_ONLY.",
                     "Stamp it with the sim clock, not with time.time()."])),
        ("""        m = PositionTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        m.type_mask = VELOCITY_ONLY
        m.velocity.x, m.velocity.y, m.velocity.z = vx, vy, vz
        m.yaw = yaw
        self.sp.publish(m)""",
         todo("the same, but with VELOCITY_ONLY and the velocity fields.")),
        ("""        hx, hy = self.home
        return [(hx, hy), (hx + self.side, hy),
                (hx + self.side, hy + self.side), (hx, hy + self.side)][i % 4]""",
         todo("return the (x, y) of corner i of a square of side self.side,",
              extra=["starting from self.home. i wraps with % 4."])),
        ("""            if self.state.mode != "OFFBOARD":
                self._request_every_second(
                    lambda: self.set_mode.call_async(
                        SetMode.Request(custom_mode="OFFBOARD")))
            elif not self.state.armed:
                self._request_every_second(
                    lambda: self.arming.call_async(
                        CommandBool.Request(value=True)))
            else:
                self._enter("TAKEOFF")""",
         todo("ask for OFFBOARD, then arm, then move on to TAKEOFF.",
              indent="            ",
              extra=["Order matters: PX4 will not accept OFFBOARD unless",
                     "setpoints are already flowing (the STREAM phase above did",
                     "that), and arming before the mode is accepted gets you an",
                     "armed aircraft in whatever mode it was in.",
                     "Use self._request_every_second(...) -- spamming a service",
                     "while the autopilot is thinking achieves nothing.",
                     "self.state.mode and self.state.armed tell you where you are."])),
    ],
    ("lab4_perception", "scripts/target_locator.py"): [
        ("""        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        u, v = det.bbox.center.position.x, det.bbox.center.position.y

        # Step 1: a pixel is a direction. In the optical frame, z is forward.
        rx, ry, rz = (u - cx) / fx, (v - cy) / fy, 1.0
        n = math.sqrt(rx * rx + ry * ry + rz * rz)
        r_opt = (rx / n, ry / n, rz / n)""",
         todo("STEP 1 -- turn the detection's centre pixel into a UNIT bearing",
              extra=["ray r_opt in the camera OPTICAL frame (z forward, x right,",
                     "y down). self.K is the row-major 3x3: fx=K[0], fy=K[4],",
                     "cx=K[2], cy=K[5]. This is K^-1 [u, v, 1]^T, normalised.",
                     "It has to be a unit vector: step 4 depends on it."])),
        ("""        lam = (self.plane_z - p_cam[2]) / r[2]
        if lam <= 0.0 or lam > self.max_range:
            return
        px = p_cam[0] + lam * r[0]
        py = p_cam[1] + lam * r[1]""",
         todo("STEP 2 -- intersect the ray with the plane z = self.plane_z.",
              extra=["lam is the scale factor that puts p_cam + lam * r on the",
                     "plane. Reject lam <= 0 (the plane is behind you) and",
                     "lam > self.max_range (the ray is nearly horizontal and the",
                     "answer is meaningless).",
                     "px, py are the target's position in map."])),
        ("""        sin_theta = max(-r[2], 1e-3)               # depression angle of the ray
        sigma_along = lam * self.sigma_angle / sin_theta
        sigma_cross = lam * self.sigma_angle
        az = math.atan2(r[1], r[0])                # ground bearing of the ray
        ca, sa = math.cos(az), math.sin(az)
        a2, c2 = sigma_along ** 2, sigma_cross ** 2
        cxx = a2 * ca * ca + c2 * sa * sa
        cyy = a2 * sa * sa + c2 * ca * ca
        cxy = (a2 - c2) * ca * sa""",
         todo("STEP 4 -- build the covariance. Because r is a unit vector, lam",
              extra=["IS the slant range, so sigma = lam * self.sigma_angle.",
                     "That error is NOT circular on the ground: along the",
                     "line-of-sight it is amplified by 1/sin(theta), where",
                     "sin(theta) = -r[2]; across it, it is not amplified at all.",
                     "Build diag(sigma_along^2, sigma_cross^2) and rotate it by",
                     "the ground bearing az = atan2(r[1], r[0]) to get",
                     "cxx, cyy, cxy.",
                     "Start with the circular version if you like -- then look at",
                     "the ellipse in RViz at a shallow look angle and fix it."])),
    ],
    ("lab4_perception", "scripts/gimbal_pointer.py"): [
        ("""        # Divide by the focal length and a pixel error becomes an angle error.
        self.err = (math.atan2(du, fx), math.atan2(dv, fy))
        self.t_err = self._now()""",
         todo("turn the pixel offsets du, dv into ANGLE errors in radians.",
              extra=["Dividing by the focal length is what makes the gains",
                     "physical: a P gain of 2.0 then means 2 rad/s of gimbal",
                     "rate per radian of pointing error.",
                     "Store them in self.err as (yaw_error, pitch_error) and",
                     "record self.t_err = self._now()."])),
        ("""        dt = 1.0 / 30.0
        out = []
        for i, e in enumerate(self.err):
            self.integ[i] = max(-self.i_limit,
                                min(self.i_limit, self.integ[i] + e * dt))
            d = (e - self.prev[i]) / dt
            u = self.kp * e + self.ki * self.integ[i] + self.kd * d
            # Clamp, and stop integrating into a clamp.
            if abs(u) > self.max_rate:
                u = math.copysign(self.max_rate, u)
                self.integ[i] -= e * dt
            out.append(u)
        self.prev = self.err""",
         todo("a PID per axis, producing out = [yaw_rate, pitch_rate].",
              extra=["self.kp, self.ki, self.kd, self.max_rate, self.i_limit.",
                     "Two things that are not optional:",
                     "  * clamp the integrator to +/- self.i_limit, and",
                     "  * when the output clamps at max_rate, do not keep",
                     "    integrating into the clamp (undo that step's",
                     "    contribution). Without this the gimbal overshoots",
                     "    every time it comes off a limit.",
                     "Remember self.prev for the derivative term."])),
    ],
    ("lab5_follow", "scripts/target_tracker.py"): [
        ("""        x = self.x
        self.x = [x[0] + dt * x[2], x[1] + dt * x[3], x[2], x[3]]
        F = [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]]
        # Piecewise-constant white acceleration. Writing Q this way rather than
        # as a diagonal guess is what makes position and velocity uncertainty
        # grow together, which is what actually happens.
        s = self.q_sigma ** 2
        d4, d3, d2 = dt ** 4 / 4.0, dt ** 3 / 2.0, dt ** 2
        Q = [[d4 * s, 0, d3 * s, 0],
             [0, d4 * s, 0, d3 * s],
             [d3 * s, 0, d2 * s, 0],
             [0, d3 * s, 0, d2 * s]]
        self.P = mat_add(mat_mul(mat_mul(F, self.P), transpose(F)), Q)""",
         todo("the PREDICT step for x = [x, y, vx, vy].",
              extra=["x- = F x   with F = [[I, dt I], [0, I]]",
                     "P- = F P F^T + Q",
                     "For Q, use piecewise-constant white acceleration with",
                     "standard deviation self.q_sigma: the position block is",
                     "dt^4/4 * s, the cross block dt^3/2 * s, the velocity block",
                     "dt^2 * s, where s = q_sigma^2. A diagonal guess also 'works'",
                     "-- try both and watch what happens to the velocity estimate",
                     "during a blackout.",
                     "mat_mul, mat_add and transpose are provided above."])),
        ("""        S = [[self.P[0][0] + R[0][0], self.P[0][1] + R[0][1]],
             [self.P[1][0] + R[1][0], self.P[1][1] + R[1][1]]]
        Si = inv2(S)
        if Si is None:
            return
        nu = [z[0] - self.x[0], z[1] - self.x[1]]
        d2 = (nu[0] * (Si[0][0] * nu[0] + Si[0][1] * nu[1]) +
              nu[1] * (Si[1][0] * nu[0] + Si[1][1] * nu[1]))
        if d2 > self.gate:
            self.n_gated += 1
            return

        # K = P H' S^-1, H' picks the first two columns of P
        PHt = [[self.P[i][0], self.P[i][1]] for i in range(4)]
        K = mat_mul(PHt, Si)
        for i in range(4):
            self.x[i] += K[i][0] * nu[0] + K[i][1] * nu[1]
        # Joseph form is not needed at this size, but subtracting K H P must be
        # done against the ORIGINAL P, so build the new one rather than editing.
        HP = [[self.P[0][j] for j in range(4)], [self.P[1][j] for j in range(4)]]
        KHP = mat_mul(K, HP)
        self.P = [[self.P[i][j] - KHP[i][j] for j in range(4)] for i in range(4)]
        self.t_last_update = now
        self.n_upd += 1""",
         todo("the UPDATE step, with a Mahalanobis gate.",
              extra=["H = [I 0], so H P H^T is just the top-left 2x2 of P.",
                     "  innovation  nu = z - H x",
                     "  covariance  S  = H P H^T + R      (inv2 is provided)",
                     "  gate        d2 = nu^T S^-1 nu  >  self.gate  -> reject,",
                     "              count it in self.n_gated, and return.",
                     "  gain        K  = P H^T S^-1",
                     "  state       x += K nu",
                     "  covariance  P -= K H P",
                     "Compute the new P from the ORIGINAL P, not in place.",
                     "Set self.t_last_update = now and count self.n_upd.",
                     "The gate is three lines and it is the difference between a",
                     "filter and a liability: without it one false positive drags",
                     "the track across the field."])),
    ],
    ("lab5_follow", "scripts/follow_guidance.py"): [
        ("""        speed = math.hypot(tvx, tvy)
        if speed > self.min_speed:
            hx, hy = tvx / speed, tvy / speed
            if self.heading is None:
                self.heading = (hx, hy)
            else:
                a = min(1.0, (1.0 / 20.0) / max(self.heading_tau, 1e-3))
                bx = self.heading[0] + a * (hx - self.heading[0])
                by = self.heading[1] + a * (hy - self.heading[1])
                n = math.hypot(bx, by)
                if n > 1e-6:
                    self.heading = (bx / n, by / n)
        # When the target stops, v_hat is undefined. Latching the last valid
        # heading keeps the aircraft where it was rather than snapping
        # overhead, and a latched bearing is a decision -- dividing by a speed
        # of zero is a crash.
        if self.heading is None:
            # Hold the bearing we are already on, at the standoff distance.
            #
            # The tempting fallback -- sit directly overhead -- was tried and
            # is wrong twice over. It throws away the look angle the whole
            # error model is built on, and it puts the camera at nadir, where
            # a top-down truck stops looking like anything COCO was trained on:
            # measured, detections went to zero the moment the aircraft arrived
            # over a stationary target, and the mission went to LOST while
            # hovering directly above a truck in plain view.
            bx, by = self.pose.x - tx, self.pose.y - ty
            n = math.hypot(bx, by)
            if n > 0.5:
                px, py = tx + self.d * bx / n, ty + self.d * by / n
            else:
                px, py = tx + self.d, ty       # any bearing beats none
        else:
            px = tx - self.d * self.heading[0]
            py = ty - self.d * self.heading[1]
        pz = self.h""",
         todo("the standoff reference: p_des = p_T - d * v_hat_T + h * z_hat.",
              extra=["self.d, self.h; the target is at (tx, ty) moving at",
                     "(tvx, tvy). Produce px, py, pz.",
                     "Guard the degenerate case: when the target's speed drops",
                     "below self.min_speed, v_hat is undefined. Latch the last",
                     "valid heading in self.heading rather than dividing by zero,",
                     "and when you have never had one, hold the bearing you are",
                     "already on at distance d. Do NOT sit directly overhead:",
                     "it throws away the look angle, and a nadir view of a",
                     "vehicle is one the detector was never trained on."])),
        ("""        sp_h = math.hypot(vx, vy)
        if sp_h > self.vmax:
            vx, vy = vx * self.vmax / sp_h, vy * self.vmax / sp_h
        pz = max(pz, self.floor)
        r = math.hypot(px, py)
        if r > self.fence:
            px, py = px * self.fence / r, py * self.fence / r""",
         todo("the clamps, applied AFTER the law and allowed to overrule it.",
              extra=["  * horizontal speed <= self.vmax (scale vx, vy together,",
                     "    or you change the direction of travel as well as its",
                     "    magnitude);",
                     "  * altitude floor self.floor: never command below it;",
                     "  * geofence self.fence: clamp the reference's radius from",
                     "    the origin.",
                     "Order matters. Safety goes last."])),
    ],
    ("capstone_follow", "scripts/mission_manager.py"): [
        ("""            self._point_gimbal(self.search_pitch,
                               self.scan_amp * math.sin(self.scan_rate * self._elapsed()))
            if self._have_track():
                self._enter("FOLLOW")""",
         todo("sweep the gimbal in yaw while searching, and leave SEARCH for",
              indent="            ",
              extra=["FOLLOW once there is a live track. self.scan_amp and",
                     "self.scan_rate shape the sweep; self.search_pitch is how",
                     "far down to look.",
                     "Sweeping the gimbal beats yawing the aircraft: it is",
                     "faster, and it does not move the vehicle while nobody",
                     "knows where the target is."])),
        ("""            if not self._have_track():
                self._enter("LOST")""",
         todo("leave FOLLOW for LOST when the track goes stale.",
              indent="            ",
              extra=["Publish nothing else here: follow_guidance owns the",
                     "setpoint topic while it is enabled, and two writers to one",
                     "topic is a fight the later publisher wins."])),
        ("""            e = self._elapsed()
            if self._have_track():
                self._enter("FOLLOW")
            elif e < self.coast_s:
                pass                                   # the filter may still recover it
            elif e < self.coast_s + self.scan_s:
                self._point_gimbal(self.search_pitch,
                                   self.scan_amp * math.sin(self.scan_rate * (e - self.coast_s)))
            else:
                self.get_logger().warn("target not reacquired; breaking off")
                self._enter("RTL")""",
         todo("the LOST ladder, with a timeout on every rung.",
              indent="            ",
              extra=["  reacquired at any point      -> FOLLOW",
                     "  for the first coast_s        -> do nothing; the filter is",
                     "                                  still coasting",
                     "  then for scan_s              -> sweep the gimbal in yaw",
                     "  after that                   -> RTL",
                     "A LOST branch whose answer is 'keep trying' is how an",
                     "aircraft ends up somewhere nobody chose."])),
    ],
}


def main():
    if os.path.isdir(SKEL):
        shutil.rmtree(SKEL)
    os.makedirs(SKEL)

    made = 0
    for pkg_dir in sorted(os.listdir(SOL)):
        if not pkg_dir.endswith("_solution"):
            continue
        base = pkg_dir[: -len("_solution")]
        src = os.path.join(SOL, pkg_dir)
        dst = os.path.join(SKEL, base)
        shutil.copytree(src, dst,
                        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))

        # Rename the package everywhere it appears.
        for root, _, files in os.walk(dst):
            for fn in files:
                p = os.path.join(root, fn)
                with open(p) as f:
                    text = f.read()
                text = text.replace(pkg_dir, base)
                text = text.replace("Reference solution for", "Skeleton for")
                with open(p, "w") as f:
                    f.write(text)
        for fn in os.listdir(os.path.join(dst, "launch")):
            new = fn.replace("_solution", "")
            if new != fn:
                os.rename(os.path.join(dst, "launch", fn),
                          os.path.join(dst, "launch", new))

        readme = os.path.join(HERE, "readmes", base + ".md")
        if os.path.exists(readme):
            shutil.copy(readme, os.path.join(dst, "README.md"))

        for (cpkg, cfile), cuts in CUTS.items():
            if cpkg != base:
                continue
            p = os.path.join(dst, cfile)
            with open(p) as f:
                text = f.read()
            for block, hint in cuts:
                n = text.count(block)
                if n != 1:
                    sys.exit(f"{cfile}: block matched {n} times, expected 1:\n"
                             f"{block[:120]}...")
                indent = block[: len(block) - len(block.lstrip(" "))]
                text = text.replace(
                    block, hint + "\n" + indent + "raise NotImplementedError")
                made += 1
            # A skeleton that claims to be a reference solution is a lie.
            text = text.replace("reference solution", "skeleton")
            with open(p, "w") as f:
                f.write(text)

    print(f"skeletons regenerated: {made} blocks cut")


if __name__ == "__main__":
    main()
