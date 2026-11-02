"""Single asynchronous costmap reader. No ROS or simulator dependency."""
import math
import threading
import time


class CostmapCache:
    def __init__(self,client,request_factory,clock=time.monotonic,period=.2,timeout=1.):
        self.client=client;self.request_factory=request_factory;self.clock=clock
        self.period=period;self.timeout=timeout;self.lock=threading.RLock()
        self.pending=None;self.started=0.;self.last_request=-math.inf
        self.message=None;self.received=0.;self.sequence=0
        self.error='waiting for costmap';self.latency=None;self.timeouts=0

    def poll(self):
        retired=None
        with self.lock:
            now=self.clock()
            if self.pending is not None:
                if now-self.started<=self.timeout:return
                retired=self.pending;self.pending=None
                self.error='costmap response timeout';self.timeouts+=1
            if retired is None and (now-self.last_request<self.period or not self.client.service_is_ready()):return
        if retired is not None:
            self.client.remove_pending_request(retired);retired.cancel()
            return
        with self.lock:
            if self.pending is not None:return
            if not self.client.service_is_ready():return
            self.started=self.last_request=self.clock()
            try:
                future=self.client.call_async(self.request_factory())
            except Exception as error:
                self.error='costmap request failed: '+str(error);return
            self.pending=future
            future.add_done_callback(self.complete)

    def complete(self,future):
        with self.lock:
            if future is not self.pending:return
            self.pending=None
            self.latency=self.clock()-self.started
            try:
                message=future.result().map
                meta=message.metadata
                if (not math.isfinite(meta.resolution) or meta.resolution<=0 or
                    meta.size_x<=0 or meta.size_y<=0 or len(message.data)!=meta.size_x*meta.size_y):
                    raise ValueError('invalid costmap dimensions')
                self.message=message;self.received=self.clock();self.sequence+=1;self.error=None
            except Exception as error:
                self.error='costmap response invalid: '+str(error)

    def snapshot(self):
        with self.lock:
            return self.message,self.received,self.sequence

    def diagnostic(self):
        with self.lock:
            now=self.clock()
            return dict(sequence=self.sequence,receive_age=now-self.received if self.message else None,
                        pending_age=now-self.started if self.pending is not None else None,
                        response_latency=self.latency,timeouts=self.timeouts,error=self.error)
