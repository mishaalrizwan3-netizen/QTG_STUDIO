"""Packaging entry point: existing application plus file-based startup diagnostics."""
import datetime
import os
from pathlib import Path
import sys
import traceback


def main():
    root = Path(sys.executable).parent if getattr(sys, 'frozen', False) else Path(__file__).resolve().parents[1]
    logs = root / 'logs'
    logs.mkdir(exist_ok=True)
    stream = (logs / ('startup_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + str(os.getpid()) + '.log')).open('a', encoding='utf-8', buffering=1)
    sys.stdout = sys.stderr = stream
    print('Executable:', sys.executable, 'Frozen:', getattr(sys, 'frozen', False), flush=True)
    try:
        if '--demo-self-test' in sys.argv:
            from tools.demo_offline_check import run
            return run(root)
        if '--brunner-test' in sys.argv and os.environ.get('QTG_DEMO_OFFLINE_TEST') == '1':
            from tools.demo_offline_check import worker
            return worker(sys.argv[sys.argv.index('--brunner-test') + 1])
        from frontend.main import cli
        return cli()
    except SystemExit:
        raise
    except BaseException:
        traceback.print_exc()
        if '--demo-self-test' not in sys.argv and '--brunner-test' not in sys.argv:
            import ctypes
            ctypes.windll.user32.MessageBoxW(0, 'QTG Studio could not start. See logs in:\n' + str(logs), 'QTG Studio startup error', 0x10)
        return 1


if __name__ == '__main__':
    sys.exit(main())
