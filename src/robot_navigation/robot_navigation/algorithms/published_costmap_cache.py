"""ROS-free cache of complete, locked Nav2 raw publications.

Jazzy GetCostmap copies the live buffer without the update mutex; it can read
the reset stage of reinflation. prepareCostmap instead holds that mutex. Use
full costmap_raw messages, not partial updates or request-time service stamps.
"""
import copy
import math
import threading
import time

class PublishedCostmapCache:
    def __init__(self,expected_frame='map',clock=time.monotonic):
        self.expected_frame=expected_frame;self.clock=clock;self.lock=threading.RLock()
        self.message=None;self.received=0.;self.sequence=0;self.stamp=-math.inf
        self.error='waiting for complete published costmap';self.rejected=0

    def accept(self,message,ros_now,max_age=1.5):
        try:
            sec=message.header.stamp.sec;ns=message.header.stamp.nanosec
            if type(sec) is not int or type(ns) is not int or not 0<=ns<10**9:
                raise ValueError('invalid published stamp')
            stamp=sec+ns*1e-9;meta=message.metadata
            if stamp<=0 or not -.05<=ros_now-stamp<max_age:raise ValueError('published costmap stamp stale/future')
            if message.header.frame_id!=self.expected_frame:raise ValueError('published costmap frame mismatch')
            if (not math.isfinite(meta.resolution) or meta.resolution<=0 or
                    meta.size_x<=0 or meta.size_y<=0 or len(message.data)!=meta.size_x*meta.size_y):
                raise ValueError('invalid published costmap dimensions')
            origin=meta.origin
            q=origin.orientation
            if not all(math.isfinite(v) for v in
                       (origin.position.x,origin.position.y,q.x,q.y,q.z,q.w)):
                raise ValueError('nonfinite published costmap origin')
            if abs(q.x)+abs(q.y)+abs(q.z)>1e-6 or abs(q.w*q.w-1.)>.01:
                raise ValueError('published costmap origin must be normalized and axis aligned')
            with self.lock:
                if stamp<=self.stamp:return False
                snapshot=copy.deepcopy(message)
                # Nav2 raw metadata.update_time may be zero. The header is the
                # complete snapshot publication time; never substitute receipt time.
                snapshot.metadata.update_time=copy.copy(message.header.stamp)
                self.message=snapshot;self.stamp=stamp;self.received=self.clock()
                self.sequence+=1;self.error=None
            return True
        except (AttributeError,ValueError,TypeError) as exc:
            with self.lock:self.error=str(exc);self.rejected+=1
            return False

    def poll(self):pass  # Compatibility with the inherited execution bridge timer.

    def snapshot(self):
        with self.lock:return self.message,self.received,self.sequence

    def diagnostic(self):
        with self.lock:
            return dict(source='published_costmap_raw',sequence=self.sequence,
                receive_age=self.clock()-self.received if self.message else None,
                source_stamp=self.stamp if self.message else None,
                rejected=self.rejected,error=self.error)
