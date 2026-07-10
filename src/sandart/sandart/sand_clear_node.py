#!/usr/bin/env python3

import threading

import rclpy
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import DR_init


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

MOVEJ_VEL = 20
MOVEJ_ACC = 20

MOVEL_VEL = 30
MOVEL_ACC = 30

GRIPPER_PULSE_TIME = 0.5


setattr(DR_init, "__dsr__id", ROBOT_ID)
setattr(DR_init, "__dsr__model", ROBOT_MODEL)


class SandClearNode(Node):

    def __init__(self):
        super().__init__("sand_clear_node", namespace=ROBOT_ID)

        setattr(DR_init, "__dsr__id", ROBOT_ID)
        setattr(DR_init, "__dsr__model", ROBOT_MODEL)
        setattr(DR_init, "__dsr__node", self)

        from DSR_ROBOT2 import (
            movej,
            movel,
            wait,
            set_digital_output,
            DR_BASE,
            DR_MV_MOD_ABS,
        )

        from DR_common2 import posj, posx

        self.movej = movej
        self.movel = movel
        self.wait = wait
        self.set_digital_output = set_digital_output
        self.posj = posj
        self.posx = posx

        self.DR_BASE = DR_BASE
        self.DR_MV_MOD_ABS = DR_MV_MOD_ABS

        self.motion_started = False
        self.motion_thread = None

        self.home_joint = self.posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0)

        self.p_grip = (374.670, -192.170, 88.130, 118.87, 179.86, 118.38)
        self.p_grip_raise = (368.730, -189.280, 222.660, 167.92, -179.93, 166.39)

        self.p_for_above = (374.890, 171.370, 231.220, 136.8, 179.82, 135.74)
        self.p_for_above_tilt = (372.610, 103.720, 231.220, 88.54, 145.26, 85.66)
        self.p_for_ground_1 = (372.610, 103.720, 142.750, 88.54, 145.26, 85.66)
        self.p_for_ground_2 = (372.610, 53.720, 102.750, 88.54, 145.26, 85.66)
        self.p_for_end = (372.610, -140.320, 102.750, 88.54, 145.26, 85.66)
        self.p_for_end_above = (372.610, -140.320, 200.040, 88.54, 145.26, 85.66)

        self.p_back_above = (374.830, -175.650, 200.040, 98.71, -173.18, 94.02)
        self.p_back_above_tilt = (378.190, -92.350, 200.040, 96.0, -143.94, 89.96)
        self.p_back_ground_1 = (378.190, -92.350, 134.110, 96.0, -143.94, 89.96)
        self.p_back_ground_2 = (378.190, -42.350, 99.110, 96.0, -143.94, 89.96)
        self.p_back_end = (378.180, 150.150, 99.110, 95.99, -143.94, 89.96)
        self.p_back_end_above = (378.180, 150.150, 200.040, 95.99, -143.94, 89.96)

        self.p_release = (374.670, -192.170, 98.130, 118.87, 179.86, 118.38)
        self.start_clear_srv = self.create_service(
            Trigger,
            "start_sand_clear",
            self.start_clear_callback,
        )

        # 실제 Sand Clear 동작 완료를 HMI에 알리는 토픽
        # namespace가 dsr01이므로 최종 토픽은 /dsr01/sand_clear_done
        self.clear_done_pub = self.create_publisher(
            Bool,
            "sand_clear_done",
            10,
        )

    def outputs_off(self):
        self.set_digital_output(-1)
        self.set_digital_output(-2)

    def gripper_open(self):
        self.outputs_off()

        self.set_digital_output(2)
        self.wait(GRIPPER_PULSE_TIME)

        self.outputs_off()
        self.wait(0.2)

    def gripper_close(self):
        self.outputs_off()

        self.set_digital_output(1)
        self.wait(GRIPPER_PULSE_TIME)

        self.outputs_off()
        self.wait(0.2)

    def execute_movej(self, target, vel=None, acc=None):
        if vel is None:
            vel = MOVEJ_VEL

        if acc is None:
            acc = MOVEJ_ACC

        return self.movej(
            target,
            vel=vel,
            acc=acc,
        )

    def execute_movel(self, target, vel=None, acc=None):
        if vel is None:
            vel = MOVEL_VEL

        if acc is None:
            acc = MOVEL_ACC

        return self.movel(
            self.posx(*target),
            vel=vel,
            acc=acc,
            ref=self.DR_BASE,
            mod=self.DR_MV_MOD_ABS,
        )
    
    def start_clear_callback(
        self,
        request,
        response,
    ):
        """서비스 요청을 받고 실제 모션은 별도 스레드에서 실행합니다."""

        if self.motion_started:
            response.success = False
            response.message = "Already Running"
            return response

        self.motion_started = True

        self.get_logger().info(
            "Sand Clear Start"
        )

        self.motion_thread = threading.Thread(
            target=self.motion_worker,
            daemon=True,
        )
        self.motion_thread.start()

        # 서비스는 즉시 반환하고, 실제 완료는 /dsr01/sand_clear_done으로 알립니다.
        response.success = True
        response.message = "Started"
        return response

    def publish_clear_done(self, success: bool):
        msg = Bool()
        msg.data = bool(success)
        self.clear_done_pub.publish(msg)

    def motion_worker(self):
        success = False

        try:
            self.run_motion()
            success = True

        except Exception as e:
            self.get_logger().error(
                f"Sand Clear Failed: {e}"
            )

        finally:
            try:
                self.outputs_off()
            except Exception:
                pass

            self.motion_started = False

            if success:
                self.get_logger().info(
                    "Sand Clear Finished"
                )

            self.publish_clear_done(success)

    def run_motion(self):
        self.wait(0.5)

        self.gripper_open()

        self.execute_movej(self.home_joint)

        self.execute_movel(
            self.p_grip,
            vel=70,
            acc=70,
        )

        self.gripper_close()

        motion_points = [
            (self.p_grip_raise, 70, 70),

            (self.p_for_above, 70, 70),
            (self.p_for_above_tilt, 70, 70),
            (self.p_for_ground_1, 50, 50),
            (self.p_for_ground_2, MOVEL_VEL, MOVEL_ACC),
            (self.p_for_end, MOVEL_VEL, MOVEL_ACC),
            (self.p_for_end_above, 50, 50),

            (self.p_back_above, 70, 70),
            (self.p_back_above_tilt, 70, 70),
            (self.p_back_ground_1, 50, 50),
            (self.p_back_ground_2, MOVEL_VEL, MOVEL_ACC),
            (self.p_back_end, MOVEL_VEL, MOVEL_ACC),
            (self.p_back_end_above, 50, 50),

            (self.p_grip_raise, 70, 70),
            (self.p_release, MOVEL_VEL, MOVEL_ACC),
        ]

        for target, vel, acc in motion_points:
            self.execute_movel(
                target,
                vel=vel,
                acc=acc,
            )

        self.gripper_open()

        self.execute_movel(
            self.p_grip_raise,
            vel=70,
            acc=70,
        )

        self.execute_movej(self.home_joint)



def main(args=None):
    rclpy.init(args=args)

    node = None
    executor = None

    try:
        node = SandClearNode()

        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()

    except KeyboardInterrupt:
        pass

    except Exception:
        pass

    finally:
        if executor is not None:
            if node is not None:
                try:
                    executor.remove_node(node)
                except Exception:
                    pass

            try:
                executor.shutdown()
            except Exception:
                pass

        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

        setattr(DR_init, "__dsr__node", None)

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()