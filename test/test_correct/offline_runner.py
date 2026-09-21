from pathlib import Path
import sys
import threading
import time

from flask import Flask, Response, jsonify, request

from vision_preview import VIEW_NAMES, render_preview


PROJECT_ROOT = Path(__file__).resolve().parents[2]


class OfflineVisionRunner:
    def __init__(self, web_port=5010):
        self.web_port = web_port
        self._closing = threading.Event()
        self._thread = None
        self._camera = None
        self._tracker = None
        self._cv2 = None
        self._lock = threading.Lock()
        self._preview = None
        self._frame_id = 0
        self._error = ""

    def start(self):
        self._prepare_vision()
        self._thread = threading.Thread(target=self._vision_loop, daemon=True)
        self._thread.start()
        self.create_app().run(host="0.0.0.0", port=self.web_port, threaded=True)

    def shutdown(self):
        self._closing.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._camera is not None:
            self._camera.release()
            self._camera = None
            self._tracker = None

    def _prepare_vision(self):
        import cv2
        import yaml
        sys.path.insert(0, str(PROJECT_ROOT))
        from perception.devices.camera import CameraManager
        from perception.algorithms.lane.tracker import LaneTracker

        settings = yaml.safe_load((PROJECT_ROOT / "config" / "settings.yaml").read_text(encoding="utf-8")) or {}
        from perception.profiles import apply_vision_profile
        settings = apply_vision_profile(settings)
        camera = CameraManager(settings)
        if not camera.initialize():
            raise RuntimeError("camera initialization failed")
        try:
            tracker = LaneTracker(settings)
            tracker.set_debug_capture_enabled(True)
        except Exception:
            camera.release()
            raise
        self._camera, self._tracker, self._cv2 = camera, tracker, cv2

    def _vision_loop(self):
        while not self._closing.is_set():
            try:
                frame = self._camera.get_front_frame()
                if frame is None:
                    time.sleep(0.02)
                    continue
                started = time.perf_counter()
                offset, intersection, _ = self._tracker.process(frame)
                process_ms = (time.perf_counter() - started) * 1000
                clean_mask = self._tracker.last_seg_mask
                if clean_mask is None:
                    time.sleep(0.02)
                    continue
                lane_state = self._tracker.last_lane_state or {}
                bev_mask = self._tracker.last_bev_mask
                with self._lock:
                    self._preview = (frame, clean_mask, bev_mask, lane_state, offset, intersection,
                                     process_ms, self._tracker.ipm.pitch_deg,
                                     self._tracker.physical_track_width_mm,
                                     self._tracker.last_original_view,
                                     self._tracker.last_debug_capture)
                    self._frame_id += 1
            except Exception as error:
                with self._lock:
                    self._error = str(error)
                time.sleep(0.1)

    def _jpeg(self, view):
        if view not in VIEW_NAMES:
            raise ValueError(f"unknown vision view: {view}")
        with self._lock:
            preview = self._preview
        if preview is None:
            return None
        debug_capture = preview[10] if len(preview) > 10 else None
        render_args = (
            view,
            preview[0],
            preview[1],
            preview[2],
            preview[3],
            preview[7],
            preview[8],
            preview[4],
            preview[6],
        )
        if len(preview) <= 9:
            return render_preview(*render_args)
        return render_preview(
            *render_args,
            original_view=preview[9],
            diagnostics={"debug_capture": debug_capture},
        )
    def _set_semantic_gate(self, enabled):
        if self._tracker is None:
            raise RuntimeError("vision tracker is unavailable")
        return self._tracker.set_semantic_gate(bool(enabled))

    def create_app(self):
        app = Flask(__name__)

        @app.after_request
        def no_cache(response):
            response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
            return response

        @app.get("/")
        def index():
            return OFFLINE_PAGE

        @app.get("/api/status")
        def status():
            with self._lock:
                return jsonify({"state": "OFFLINE_RUNNING", "error": self._error,
                                "vision_frame_id": self._frame_id, "vision_views": list(VIEW_NAMES),
                                "semantic_gate_available": bool(
                                    self._tracker is not None and self._tracker.semantic_engine is not None
                                ),
                                "semantic_gate_enabled": bool(
                                    self._tracker is not None and self._tracker.semantic_gate_enabled
                                )})

        @app.get("/api/vision")
        def vision():
            try:
                image = self._jpeg(request.args.get("view", "overlay"))
            except ValueError as error:
                return jsonify({"ok": False, "error": str(error)}), 400
            if image is None:
                return jsonify({"ok": False, "error": "vision preview is unavailable"}), 404
            return Response(image, mimetype="image/jpeg")

        @app.post("/api/semantic-gate")
        def semantic_gate():
            try:
                data = request.get_json(force=True) or {}
                return jsonify({"ok": True, "enabled": self._set_semantic_gate(data.get("enabled", False))})
            except Exception as error:
                return jsonify({"ok": False, "error": str(error)}), 400

        return app


OFFLINE_PAGE = """<!doctype html>
<meta charset="utf-8"><title>单机视觉回正</title>
<style>body{font:16px sans-serif;max-width:960px;margin:24px auto}pre{height:90px;overflow:auto;background:#111;color:#ddd;padding:12px}.preview{min-height:420px;background:#10151d;display:flex;align-items:center;justify-content:center;border-radius:6px;overflow:hidden}.preview img{display:none;max-width:100%;height:auto}.preview span{color:#aab6c4}</style>
<h1>单机视觉回正</h1><p>状态：<b id="state">--</b>　错误：<span id="error">--</span></p>
<select id="visionView" onchange="refreshVision()">
<option value="overlay">原图模板线与中心线</option>
<option value="binary">边缘检测二值化结果</option>
<option value="hough">原图融合 Hough 线段</option>
<option value="lane_bev">平行坐标系模板匹配</option>
<option value="ground_bev">地面坐标系车道与中心线</option>
</select> <label><input id="semanticGate" type="checkbox" onchange="setSemanticGate(this.checked)"> Semantic gate</label> <span id="semanticGateState">--</span>
<div class="preview"><img id="visionImage"><span id="visionHint">等待相机和推理结果...</span></div><pre id="log"></pre>
<script>let u=null,last=0;async function setSemanticGate(enabled){await fetch('/api/semantic-gate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled})});}async function refreshVision(){let i=document.getElementById('visionImage'),h=document.getElementById('visionHint'),v=document.getElementById('visionView').value,r=await fetch('/api/vision?view='+v+'&t='+Date.now());if(r.ok){if(u)URL.revokeObjectURL(u);u=URL.createObjectURL(await r.blob());i.src=u;i.style.display='block';h.style.display='none';}}async function refresh(){let d=await (await fetch('/api/status')).json();state.textContent=d.state;error.textContent=d.error||'--';let g=document.getElementById('semanticGate');g.checked=Boolean(d.semantic_gate_enabled);g.disabled=!d.semantic_gate_available;document.getElementById('semanticGateState').textContent=d.semantic_gate_available?(d.semantic_gate_enabled?'enabled':'disabled'):'unavailable';if(d.vision_frame_id!==last){last=d.vision_frame_id;refreshVision();}}setInterval(refresh,500);refresh();</script>"""


if __name__ == "__main__":
    runner = OfflineVisionRunner()
    try:
        runner.start()
    finally:
        runner.shutdown()
