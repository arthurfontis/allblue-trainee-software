"""Nó de controle dos motores do AUV.

Publica o PWM dos 7 motores em /motors (Int32MultiArray) e oferece duas actions:
  - test_motor:      testa um motor por vez (2 s)
  - test_all_motors: testa cada motor (2 s cada) e depois todos juntos (2 s)

Só um teste roda por vez: se um teste estiver em andamento, qualquer novo pedido é recusado.
Ao terminar, cancelar ou falhar, todos os motores voltam para o PWM neutro (1500).
"""

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from motor_test_interfaces.action import TestAllMotors, TestMotor

PWM_MIN = 1000
PWM_NEUTRAL = 1500
PWM_MAX = 2000
FORCE_MAX = 10.0  # N em PWM_MAX (e -10 N em PWM_MIN)


def pwm_to_force(pwm):
    """Relação linear: 1000 -> -10 N, 1500 -> 0 N, 2000 -> +10 N."""
    return (pwm - PWM_NEUTRAL) / (PWM_MAX - PWM_NEUTRAL) * FORCE_MAX


class MotorControlNode(Node):

    def __init__(self):
        super().__init__('motor_control_node')

        self.declare_parameter('num_motors', 7)
        self.declare_parameter('test_pwm', 1600)
        self.declare_parameter('test_duration', 2.0)
        self.declare_parameter('publish_rate', 10.0)

        self.num_motors = self.get_parameter('num_motors').value
        self.test_pwm = self.get_parameter('test_pwm').value
        self.test_duration = self.get_parameter('test_duration').value
        publish_rate = self.get_parameter('publish_rate').value

        if not PWM_MIN <= self.test_pwm <= PWM_MAX:
            raise ValueError(f'test_pwm precisa estar entre {PWM_MIN} e {PWM_MAX}')

        self.pwm = [PWM_NEUTRAL] * self.num_motors
        self.busy = False
        self.lock = threading.Lock()

        self.motors_pub = self.create_publisher(Int32MultiArray, '/motors', 10)
        # Publica o estado atual periodicamente (neutro quando não há teste)
        self.create_timer(1.0 / publish_rate, self.publish_pwm)

        group = ReentrantCallbackGroup()
        self.test_motor_server = ActionServer(
            self, TestMotor, 'test_motor',
            execute_callback=self.execute_test_motor,
            goal_callback=self.goal_test_motor,
            cancel_callback=self.cancel_callback,
            callback_group=group)
        self.test_all_server = ActionServer(
            self, TestAllMotors, 'test_all_motors',
            execute_callback=self.execute_test_all,
            goal_callback=self.goal_test_all,
            cancel_callback=self.cancel_callback,
            callback_group=group)

        self.get_logger().info(
            f'Pronto: {self.num_motors} motores, PWM de teste {self.test_pwm} '
            f'({pwm_to_force(self.test_pwm):+.1f} N), {self.test_duration:.1f} s por teste')

    # ---------- PWM ----------

    def publish_pwm(self):
        msg = Int32MultiArray()
        msg.data = list(self.pwm)
        self.motors_pub.publish(msg)

    def set_motors(self, active_motors):
        """Coloca os motores ativos no PWM de teste e o resto em neutro."""
        self.pwm = [self.test_pwm if i in active_motors else PWM_NEUTRAL
                    for i in range(self.num_motors)]
        self.publish_pwm()

    def stop_all(self):
        self.set_motors([])

    # ---------- Controle de concorrência ----------

    def try_reserve(self):
        with self.lock:
            if self.busy:
                return False
            self.busy = True
            return True

    def release(self):
        self.stop_all()
        with self.lock:
            self.busy = False

    # ---------- Goal / cancel ----------

    def goal_test_motor(self, goal_request):
        motor = goal_request.motor_id
        if not 0 <= motor < self.num_motors:
            self.get_logger().warn(f'Motor {motor} inválido (use 0 a {self.num_motors - 1})')
            return GoalResponse.REJECT
        if not self.try_reserve():
            self.get_logger().warn(f'Teste do motor {motor} recusado: outro teste em andamento')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def goal_test_all(self, goal_request):
        if not self.try_reserve():
            self.get_logger().warn('Teste completo recusado: outro teste em andamento')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Pedido de cancelamento recebido')
        return CancelResponse.ACCEPT

    # ---------- Execução ----------

    def run_step(self, goal_handle, active_motors, make_feedback):
        """Aciona os motores por test_duration segundos.

        Retorna False se o teste foi cancelado no meio.
        """
        self.set_motors(active_motors)
        start = time.monotonic()
        while True:
            elapsed = time.monotonic() - start
            if goal_handle.is_cancel_requested or not rclpy.ok():
                return False
            goal_handle.publish_feedback(make_feedback(elapsed))
            if elapsed >= self.test_duration:
                return True
            time.sleep(0.1)

    def execute_test_motor(self, goal_handle):
        motor = goal_handle.request.motor_id
        result = TestMotor.Result()
        self.get_logger().info(f'Testando motor {motor}')

        def feedback(elapsed):
            fb = TestMotor.Feedback()
            fb.active_motor = motor
            fb.elapsed_time = float(min(elapsed, self.test_duration))
            fb.status = f'Motor {motor} ativo'
            return fb

        try:
            finished = self.run_step(goal_handle, [motor], feedback)
            self.stop_all()
            if finished:
                goal_handle.succeed()
                result.success = True
                result.message = f'Motor {motor} testado com sucesso'
            else:
                goal_handle.canceled()
                result.success = False
                result.message = f'Teste do motor {motor} cancelado'
        except Exception as e:
            goal_handle.abort()
            result.success = False
            result.message = f'Falha no teste do motor {motor}: {e}'
        finally:
            self.release()

        self.get_logger().info(result.message)
        return result

    def execute_test_all(self, goal_handle):
        result = TestAllMotors.Result()
        # Etapas: cada motor sozinho e, por último, todos juntos
        steps = [[i] for i in range(self.num_motors)]
        steps.append(list(range(self.num_motors)))
        tested = []
        self.get_logger().info('Iniciando teste completo')

        try:
            for index, active in enumerate(steps):
                step_number = index + 1
                if len(active) == 1:
                    status = f'Etapa {step_number}/{len(steps)}: motor {active[0]}'
                else:
                    status = f'Etapa {step_number}/{len(steps)}: todos os motores'

                def feedback(elapsed, active=active, status=status, step=step_number):
                    fb = TestAllMotors.Feedback()
                    fb.active_motors = active
                    fb.tested_motors = list(tested)
                    fb.step = step
                    fb.total_steps = len(steps)
                    fb.status = status
                    return fb

                if not self.run_step(goal_handle, active, feedback):
                    self.stop_all()
                    goal_handle.canceled()
                    result.success = False
                    result.message = f'Teste completo cancelado na etapa {step_number}'
                    break
                if len(active) == 1:
                    tested.append(active[0])
            else:
                self.stop_all()
                goal_handle.succeed()
                result.success = True
                result.message = 'Teste completo concluído com sucesso'
        except Exception as e:
            goal_handle.abort()
            result.success = False
            result.message = f'Falha no teste completo: {e}'
        finally:
            self.release()

        self.get_logger().info(result.message)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Garante PWM neutro ao desligar (se o contexto ROS ainda estiver ativo)
        if rclpy.ok():
            node.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
