import threading
import time
from sitl_simulator import SITLSimulator
from sitl_rate_controller import RateModeController, FakeSerial


def create_virtual_link():
    s1 = FakeSerial(timeout=0.01)
    s2 = FakeSerial(timeout=0.01)
    s1.set_peer(s2)
    s2.set_peer(s1)
    return s1, s2


def main(duration_sec=60):
    # Create virtual serial with two ends
    sim_serial, ctrl_serial = create_virtual_link()

    # Build simulator and controller with injected virtual serial
    simulator = SITLSimulator(serial_port=sim_serial)
    controller = RateModeController(serial_port=ctrl_serial)

    # Run in separate threads
    t_sim = threading.Thread(target=simulator.run, kwargs={'duration_sec': duration_sec, 'show_plot': True}, daemon=True)
    t_ctrl = threading.Thread(target=controller.run, kwargs={'duration_sec': duration_sec}, daemon=True)

    t_sim.start()
    t_ctrl.start()

    t_sim.join()
    t_ctrl.join()

    simulator.save_log('sitl_run_log.json')
    print('SITL run complete. log saved sitl_run_log.json')


if __name__ == '__main__':
    main(duration_sec=120)
