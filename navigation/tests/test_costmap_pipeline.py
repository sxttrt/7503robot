"""Offline delayed/lost/reordered map responses; no ROS nodes."""
import sys
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from costmap_cache import CostmapCache


def message(value=0):
    return NS(metadata=NS(resolution=.025,size_x=2,size_y=2),data=[value]*4)


class Client:
    def __init__(self):self.calls=[];self.removed=[]
    def service_is_ready(self):return True
    def call_async(self,request):
        f=Future();self.calls.append(f);return f
    def remove_pending_request(self,f):self.removed.append(f)


class PipelineTests(unittest.TestCase):
    def cache(self):
        clock=NS(now=0.);client=Client()
        return CostmapCache(client,lambda:object(),clock=lambda:clock.now),client,clock
    def test_pending_service_never_blocks_snapshot_or_starts_duplicate_request(self):
        cache,client,clock=self.cache();cache.poll()
        for i in range(40):
            clock.now=i*.02;cache.poll();self.assertEqual(cache.snapshot(),(None,0.,0))
        self.assertEqual(len(client.calls),1)
        client.calls[0].set_result(NS(map=message()))
        self.assertEqual(cache.snapshot()[2],1)
    def test_delayed_response_keeps_previous_snapshot_and_records_latency(self):
        cache,client,clock=self.cache();cache.poll();client.calls[-1].set_result(NS(map=message()))
        clock.now=.2;cache.poll();clock.now=.8
        self.assertEqual(cache.snapshot()[0].data,[0]*4)
        client.calls[-1].set_result(NS(map=message(254)))
        self.assertEqual(cache.snapshot()[0].data,[254]*4)
        self.assertAlmostEqual(cache.diagnostic()['response_latency'],.6)
    def test_timeout_cancel_and_late_reply_cannot_overwrite_newer_request(self):
        cache,client,clock=self.cache();cache.poll();old=client.calls[-1]
        clock.now=1.1;cache.poll();self.assertEqual(cache.diagnostic()['timeouts'],1)
        self.assertIsNone(cache.pending);self.assertIsNone(cache.snapshot()[0])
        cache.poll();new=client.calls[-1]
        cache.complete(old);self.assertIs(cache.pending,new)
        new.set_result(NS(map=message(254)));self.assertEqual(cache.snapshot()[2],1)
    def test_duplicate_reply_does_not_refresh_age_or_sequence(self):
        cache,client,clock=self.cache();cache.poll();future=client.calls[-1]
        future.set_result(NS(map=message()));clock.now=.9;cache.complete(future)
        self.assertEqual(cache.snapshot()[1:],(0.,1))
    def test_invalid_map_does_not_replace_previous_good_snapshot(self):
        cache,client,clock=self.cache();cache.poll();client.calls[-1].set_result(NS(map=message()))
        clock.now=.2;cache.poll();bad=message();bad.data=[]
        client.calls[-1].set_result(NS(map=bad))
        self.assertEqual(cache.snapshot()[2],1);self.assertIn('invalid',cache.diagnostic()['error'])
    def test_control_source_no_longer_requests_or_stops_for_valid_map_checks(self):
        import ast
        root=Path(__file__).resolve().parents[1]
        tree=ast.parse((root/'scripts/mission_v3.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MissionV3')
        get=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='get_grid')
        self.assertFalse(any(isinstance(n,ast.Attribute) and n.attr in ('request','call_async','spin') for n in ast.walk(get)))


if __name__=='__main__':unittest.main()
