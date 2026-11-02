from pathlib import Path
import unittest

PROJECT = Path(__file__).resolve().parents[1]

class OldV6NetworkContract(unittest.TestCase):
    def test_v6_wildcard_binding_main_and_preflight(self):
        s=(PROJECT/'start_v12.sh').read_text(encoding='utf-8')
        self.assertIn('RK_BIND_IP="${RK_BIND_IP:-0.0.0.0}"',s)
        self.assertIn('ARGS=(main.py --bind "$RK_BIND_IP"',s)
        self.assertIn('--source-ip "$FPGA_IP"',s)
        self.assertIn('--signal-ip "$FPGA_IP"',s)
        self.assertIn('--local-ip "$RK_BIND_IP"',s)
    def test_route_get_from_is_accepted(self):
        s=(PROJECT/'setup_fpga_net.sh').read_text(encoding='utf-8')
        self.assertIn('" from $LOCAL_IP "',s)
        self.assertIn('" src $LOCAL_IP"',s)
        self.assertIn('ip -4 route get "$FPGA_IP" from "$LOCAL_IP"',s)
    def test_preflight_and_compare_wildcard(self):
        for fn in ('fpga_pair_preflight.py','fpga_protocol_compare.py'):
            self.assertIn("p.add_argument('--local-ip',default='0.0.0.0')",(PROJECT/fn).read_text(encoding='utf-8'))

if __name__ == '__main__': unittest.main()
