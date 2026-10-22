"""Make an annotated replay from all recorded paired RGB-D frames."""
import argparse
import json
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image,ImageDraw

from task_runner import digest,write


def main(directory):
    directory=Path(directory).resolve()
    output=directory/"camera-replay.gif"
    if output.exists():
        raise FileExistsError(output)
    rows=[json.loads(s) for s in (directory/"trajectory.jsonl").read_text().splitlines()]
    frames=[]
    with np.load(directory/"observations.npz",allow_pickle=False) as source:
        cameras=json.loads(str(source["camera_json"]))
        for rgb,depth,camera in zip(source["rgb"],source["depth_m"],cameras,strict=True):
            value=np.clip((depth-.1)/.9,0,1)
            near=np.array([25,40,145]);middle=np.array([50,200,170]);far=np.array([250,225,80])
            color=np.where((value<.5)[...,None],near+(middle-near)*(value*2)[...,None],
                           middle+(far-middle)*((value-.5)*2)[...,None]).clip(0,255).astype(np.uint8)
            canvas=Image.new("RGB",(960,410),"#151820")
            canvas.paste(Image.fromarray(rgb).resize((480,360),Image.Resampling.NEAREST),(0,40))
            canvas.paste(Image.fromarray(color).resize((480,360),Image.Resampling.NEAREST),(480,40))
            draw=ImageDraw.Draw(canvas)
            t=camera["simulation_time_s"]
            nearest=min(rows,key=lambda r:abs(r["t_sim_s"]-t))
            draw.text((8,5),f"Recorded RGB | t={t:.3f}s | {nearest['stage']}",fill="white")
            draw.text((488,5),"Recorded depth: blue=0.1m, green=0.55m, yellow=1.0m",fill="white")
            draw.text((8,22),directory.name,fill="white")
            frames.append(np.asarray(canvas))
    assert frames
    imageio.mimsave(output,frames,duration=200,loop=0)
    Image.fromarray(frames[-1]).save(directory/"camera-replay-final.png")
    manifest={"frames":len(frames),"nominal_fps":5,"uses_all_recorded_frames":True,
              "source_observations_sha256":digest(directory/"observations.npz"),
              "source_trajectory_sha256":digest(directory/"trajectory.jsonl"),
              "source_result_sha256":digest(directory/"result.json"),
              "replay_sha256":digest(output),"replay_code_sha256":digest(Path(__file__)),
              "scope":"Recorded camera data playback, not reconstructed overview or physics re-execution"}
    write(directory/"camera_replay_manifest.json",manifest)
    print(json.dumps({"output":str(output),**manifest}),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode",type=Path,required=True)
    main(parser.parse_args().episode)
