"""Optional manual motion-debug gates. QR always uses the framework topic."""
from .mission_node import ROSBackend


class ManualBackend(ROSBackend):
    def __init__(self,node,config):
        super().__init__(node,config)
        self.waiting=None

    def start(self,kind,request,payload,callback):
        phase=payload.get('phase')
        gate=(phase in ('NAV_START','NAV_END') or kind=='docking')
        if gate:
            self.waiting=(kind,request,payload,callback)
            self.navigation.node.get_logger().info(f'等待开关: {phase}；发布 /rack/next true 一次')
        else:super().start(kind,request,payload,callback)

    def release(self):
        if self.waiting is None:return False
        kind,request,payload,callback=self.waiting;self.waiting=None
        super().start(kind,request,payload,callback)
        return True

    def cancel(self,kind,request):
        if self.waiting and self.waiting[1]==request:self.waiting=None
        else:super().cancel(kind,request)
