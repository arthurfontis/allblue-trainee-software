"""Interface gráfica (Tkinter) para testar os motores do AUV.

- Um botão por motor (action test_motor)
- Um botão para o teste completo (action test_all_motors)
- Mostra o estado de cada teste e o PWM atual publicado em /motors

O ROS roda em uma thread separada; a interface só é atualizada pela thread
do Tkinter, lendo uma fila de eventos.
"""

import queue
import threading
import tkinter as tk

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from motor_test_interfaces.action import TestAllMotors, TestMotor

NUM_MOTORS = 7
PWM_NEUTRAL = 1500


class GuiNode(Node):
    """Parte ROS da interface: action clients e assinatura de /motors."""

    def __init__(self, events):
        super().__init__('motor_test_gui')
        self.events = events
        self.goal_handle = None

        self.test_motor_client = ActionClient(self, TestMotor, 'test_motor')
        self.test_all_client = ActionClient(self, TestAllMotors, 'test_all_motors')
        self.create_subscription(Int32MultiArray, '/motors', self.motors_callback, 10)

    def motors_callback(self, msg):
        self.events.put(('pwm', list(msg.data)))

    # ---------- Teste individual ----------

    def send_test_motor(self, motor):
        if not self.test_motor_client.wait_for_server(timeout_sec=1.0):
            self.events.put(('motor', motor, 'Falha: servidor indisponível', 'red'))
            return
        goal = TestMotor.Goal()
        goal.motor_id = motor
        self.events.put(('motor', motor, 'Enviando...', 'gray'))
        future = self.test_motor_client.send_goal_async(
            goal, feedback_callback=lambda fb: self.motor_feedback(motor, fb))
        future.add_done_callback(lambda f: self.motor_goal_response(motor, f))

    def motor_feedback(self, motor, feedback_msg):
        fb = feedback_msg.feedback
        self.events.put(('motor', motor, f'Testando... {fb.elapsed_time:.1f} s', 'orange'))

    def motor_goal_response(self, motor, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.events.put(('motor', motor, 'Recusado (teste em andamento)', 'red'))
            return
        self.goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda f: self.motor_result(motor, f))

    def motor_result(self, motor, future):
        result = future.result().result
        if result.success:
            self.events.put(('motor', motor, 'Concluído', 'green'))
        else:
            self.events.put(('motor', motor, result.message, 'red'))
        self.goal_handle = None

    # ---------- Teste completo ----------

    def send_test_all(self):
        if not self.test_all_client.wait_for_server(timeout_sec=1.0):
            self.events.put(('all', 'Falha: servidor indisponível', 'red', []))
            return
        self.events.put(('all', 'Enviando...', 'gray', []))
        future = self.test_all_client.send_goal_async(
            TestAllMotors.Goal(), feedback_callback=self.all_feedback)
        future.add_done_callback(self.all_goal_response)

    def all_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.events.put(('all', fb.status, 'orange', list(fb.tested_motors)))

    def all_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.events.put(('all', 'Recusado (teste em andamento)', 'red', None))
            return
        self.goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.all_result)

    def all_result(self, future):
        result = future.result().result
        color = 'green' if result.success else 'red'
        tested = list(range(NUM_MOTORS)) if result.success else None
        self.events.put(('all', result.message, color, tested))
        self.goal_handle = None

    # ---------- Cancelar ----------

    def cancel(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()


class MotorTestApp:
    """Janela Tkinter."""

    def __init__(self, root, node, events):
        self.root = root
        self.node = node
        self.events = events
        root.title('All Blue — Teste dos Motores')

        # Teste individual
        frame_ind = tk.LabelFrame(root, text='Teste individual', padx=8, pady=8)
        frame_ind.pack(fill='x', padx=10, pady=5)
        self.motor_status = []
        for i in range(NUM_MOTORS):
            tk.Button(frame_ind, text=f'Motor {i}', width=10,
                      command=lambda m=i: node.send_test_motor(m)).grid(row=i, column=0, pady=2)
            label = tk.Label(frame_ind, text='Aguardando', width=32, anchor='w', fg='gray')
            label.grid(row=i, column=1, padx=8)
            self.motor_status.append(label)

        # Teste completo
        frame_all = tk.LabelFrame(root, text='Teste completo', padx=8, pady=8)
        frame_all.pack(fill='x', padx=10, pady=5)
        tk.Button(frame_all, text='Testar todos', width=14,
                  command=node.send_test_all).grid(row=0, column=0)
        tk.Button(frame_all, text='Cancelar', width=10,
                  command=node.cancel).grid(row=0, column=1, padx=8)
        self.all_status = tk.Label(frame_all, text='Aguardando', anchor='w', fg='gray')
        self.all_status.grid(row=1, column=0, columnspan=2, sticky='w', pady=(6, 0))
        self.tested_label = tk.Label(frame_all, text='Motores testados: -', anchor='w')
        self.tested_label.grid(row=2, column=0, columnspan=2, sticky='w')

        # PWM atual em /motors
        frame_pwm = tk.LabelFrame(root, text='PWM atual (/motors)', padx=8, pady=8)
        frame_pwm.pack(fill='x', padx=10, pady=5)
        self.pwm_labels = []
        for i in range(NUM_MOTORS):
            tk.Label(frame_pwm, text=f'M{i}').grid(row=0, column=i, padx=4)
            label = tk.Label(frame_pwm, text=str(PWM_NEUTRAL), width=5, relief='sunken')
            label.grid(row=1, column=i, padx=4)
            self.pwm_labels.append(label)

        self.root.after(100, self.process_events)

    def process_events(self):
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            kind = event[0]
            if kind == 'motor':
                _, motor, text, color = event
                self.motor_status[motor].config(text=text, fg=color)
            elif kind == 'all':
                _, text, color, tested = event
                self.all_status.config(text=text, fg=color)
                if tested is not None:
                    names = ', '.join(str(m) for m in tested) or '-'
                    self.tested_label.config(text=f'Motores testados: {names}')
            elif kind == 'pwm':
                for label, value in zip(self.pwm_labels, event[1]):
                    label.config(text=str(value),
                                 bg='orange' if value != PWM_NEUTRAL else 'white')
        self.root.after(100, self.process_events)


def main(args=None):
    rclpy.init(args=args)
    events = queue.Queue()
    node = GuiNode(events)

    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    threading.Thread(target=spin, daemon=True).start()

    root = tk.Tk()
    MotorTestApp(root, node, events)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
