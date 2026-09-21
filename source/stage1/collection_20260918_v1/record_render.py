"""Manual per-render acquisition using the installed Camera NEW_FRAME path."""
class ManualRendering:
    def __init__(self,products):
        import carb,omni.graph.core as og,omni.usd
        from omni.syntheticdata import _syntheticdata
        self.settings=carb.settings.get_settings()
        self.old_setting=self.settings.get('/omni/replicator/disableAnnotatorGate')
        self.settings.set('/omni/replicator/disableAnnotatorGate',True)
        self.controller=og.Controller()
        self.graph=self.controller.graph('/Render/PostProcess/SDGPipeline')
        node=self.controller.node('/Render/PostProcess/SDGPipeline/DispatchSync')
        self.gate=og.AttributeValueHelper(node.get_attribute('inputs:enabled'))
        self.old_gate=self.gate.get();assert type(self.old_gate) is bool
        self.gate.set(False,update_usd=False);assert self.gate.get() is False
        self.products={str(p.path) for p in products}
        self.events={p:0 for p in self.products};self.error=None
        self.pending=None
        self.after_frame=None
        self.sdg=_syntheticdata.acquire_syntheticdata_interface()
        self.subscription=omni.usd.get_context().get_rendering_event_stream().create_subscription_to_pop_by_type(
            int(omni.usd.StageRenderingEventType.NEW_FRAME),self.on_frame,name='stage1_native_manual_capture',order=1000)

    def on_frame(self,event):
        try:
            parsed=self.sdg.parse_rendered_simulation_event(event.payload['product_path_handle'],event.payload['results'])
            path=str(parsed[0])
            if path in self.products:
                self.controller.evaluate_sync(graph_id=self.graph)
                self.events[path]+=1
                if self.after_frame:self.after_frame(path)
                if self.pending is not None:
                    future,targets=self.pending
                    if not future.done() and all(self.events[p]>=n for p,n in targets.items()):future.set_result(dict(self.events))
        except Exception as exc:self.error=repr(exc)

    def check(self):
        if self.error:raise RuntimeError('Manual native sensor acquisition: '+self.error)

    def next_complete(self):
        import asyncio
        assert self.pending is None or self.pending[0].done()
        future=asyncio.get_running_loop().create_future()
        self.pending=(future,{p:n+1 for p,n in self.events.items()})
        return future

    def close(self):
        self.subscription=None
        self.gate.set(self.old_gate,update_usd=False)
        if self.old_setting is None:self.settings.destroy_item('/omni/replicator/disableAnnotatorGate')
        else:self.settings.set('/omni/replicator/disableAnnotatorGate',self.old_setting)
