from __future__ import print_function
from __future__ import unicode_literals
import time
import re

from tiny_test_fw import DUT, App, TinyFW
from ttfw_bl import BL602App, BL602DUT


@TinyFW.test_method(app=BL602App.BL602App, dut=BL602DUT.BL602TyMbDUT, test_suite_name='bl602_demo_noconnectivity_tc')
def bl602_demo_noconnectivity_tc(env, extra_data):
    # first, flash dut
    # then, test
    dut = env.get_dut("port0", "fake app path")
    print('Flashing app')
    dut.flash_app(env.log_path, env.get_variable('flash'))
    print('Starting app')
    dut.start_app()

    try:
        dut.expect("Booting BL602 Chip...", timeout=0.5)
        print('BL602 booted')
        dut.expect('Init CLI with event Driven', timeout=0.5)
        print('BL602 CLI init done')
        time.sleep(0.1)

        print('To reboot BL602')
        dut.write('reboot')
        dut.expect("Booting BL602 Chip...", timeout=0.5)
        print('BL602 rebooted')
        time.sleep(0.2)

        dut.write('sysver')
        time.sleep(0.5)
        left_memory = dut.expect(re.compile(r"Heap left: (\d+)"), timeout=2)
        print(f'left_memory is {left_memory[0]} Bytes')
        if int(left_memory[0]) < 230000 :
            print(f'left memory is too small {left_memory[0]}')
            raise
        else:
            print(f'left memory is big enough')

        dut.halt()

    except DUT.ExpectTimeout:
        print('ENV_TEST_FAILURE: BL602 example test failed')
        raise


if __name__ == '__main__':
    bl602_demo_noconnectivity_tc()
