import unittest,sys,struct
from pathlib import Path
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from brunner_interface import BrunnerInterface
class OwnershipTests(unittest.TestCase):
    def device(self):
        device=BrunnerInterface();device.sock=Mock();device.sock.recv.return_value=bytes([0,0,0]);return device
    def packets(self,device):
        return [struct.unpack('<III',call.args[0]) for call in device.sock.send.call_args_list if len(call.args[0])==12]
    def test_release_clears_axis_mask_preserving_each_override_id(self):
        device=self.device();self.assertTrue(device.set_overrides(7,False))
        self.assertEqual(self.packets(device),[(0xD1,0,1),(0xD1,0,3),(0xD1,0,4)])
    def test_enable_keeps_the_selected_axes(self):
        device=self.device();self.assertTrue(device.set_overrides(7,True))
        self.assertEqual(self.packets(device),[(0xD1,7,1),(0xD1,7,3),(0xD1,7,4)])
    def test_physical_forwarding_releases_only_the_forwarding_override(self):
        device=self.device();self.assertTrue(device.set_position_forwarding())
        self.assertEqual(self.packets(device),[(0xD1,0,6)])
    def test_manual_cleanup_never_commands_an_actuator_position(self):
        device=self.device();device.prepare_manual_mode()
        self.assertTrue(all(struct.unpack_from('<III',call.args[0])[2]!=0x80 for call in device.sock.send.call_args_list))
        self.assertEqual(self.packets(device)[-4:],[(0xD1,0,1),(0xD1,0,3),(0xD1,0,4),(0xD1,0,6)])
    def test_negative_ack_stops_replay_configuration_before_motor_settings(self):
        device=self.device();device.sock.recv.return_value=bytes([0,0,1])
        with self.assertRaisesRegex(RuntimeError,'acknowledge'):
            device.configure_replay_mode()
        self.assertTrue(all(packet[0]==0xD1 for packet in self.packets(device)))
if __name__=='__main__':unittest.main()
