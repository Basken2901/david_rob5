"""Run once in Isaac's Script Editor with playback stopped; saves embedded RGBD source."""
from pathlib import Path
import omni.graph.core as og
import omni.usd

repo = Path.home() / "rob5/david_rob5"
source = (repo / "scripts/david_rgbd_inline_optimized.py").read_text()
compile(source, "<DAVID_RGBD>", "exec")
stage = omni.usd.get_context().get_stage()
assert stage is not None, "Open the DAVID USD first"
node = "/_Graph_DAVID_RGBD/script_node"
assert stage.GetPrimAtPath(node).IsValid(), "Missing DAVID RGBD Script Node"
stage.SetEditTarget(stage.GetRootLayer())
og.Controller.set(og.Controller.attribute(node + ".inputs:script"), source)
og.Controller.set(og.Controller.attribute(node + ".inputs:usePath"), False)
og.Controller.set(og.Controller.attribute(node + ".state:omni_initialized"), False)
saved = og.Controller.get(og.Controller.attribute(node + ".inputs:script"))
assert saved == source, "Embedded source did not match"
compile(saved, "<DAVID_RGBD>", "exec")
print("Installed:", len(saved), "characters; syntax OK. Save USD, reload script and restart playback.")
