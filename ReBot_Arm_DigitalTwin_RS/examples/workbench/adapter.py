#!/usr/bin/env python3
"""HTTP protocol adapter template. Replace infer()/plan() with your own models.

Run --demo for a protocol self-check: empty detections + a one-second wait.
This file does not connect to ROS/CAN or implement recognition/planning.
"""
import argparse
import json
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def infer(request):
    # request['frame']: JPEG/PNG base64 + stamp_ms + frame_id.
    # Load your ONNX/PT/SDK here; return measured objects in a declared frame.
    raise NotImplementedError('请把 infer() 接到你的识别算法；--demo仅测试协议')


def plan(request):
    # request: task, robot.actual/pose/calibration_id, observation, model.
    # Return inert m/rad/s program data; never call robot drivers here.
    raise NotImplementedError('请把 plan() 接到你的任务/路径规划算法；--demo仅测试协议')


def demo_program():
    return dict(schema_version='1.0',robot_model='b601_rs',base_frame='base_link',
        calibration_id='rs-identity-v1',units=dict(position='m',angle='rad',time='s'),
        name='协议自检：只等待，不移动',steps=[dict(type='wait',seconds=1,label='仅协议联调，不控制运动')])


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--port',type=int,default=8088)
    parser.add_argument('--demo',action='store_true');args=parser.parse_args()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_):pass  # Do not log frames or API keys.
        def reply(self,status,data):
            encoded=json.dumps(data,ensure_ascii=False).encode();self.send_response(status)
            self.send_header('Content-Type','application/json; charset=utf-8');self.send_header('Content-Length',str(len(encoded)))
            self.end_headers();self.wfile.write(encoded)
        def do_GET(self):
            if self.path in ('/health','/v1/models'):
                self.reply(200,dict(ok=True,demo=args.demo,data=[dict(id='protocol-demo',object='model')]))
            else:self.reply(404,dict(error='unknown route'))
        def do_POST(self):
            try:
                length=int(self.headers.get('Content-Length',0))
                if not 0<length<=4*1024*1024:raise ValueError('body size')
                request=json.loads(self.rfile.read(length))
                envelope=dict(schema_version='1.0',request_id=request.get('request_id'),stamp_ms=time.time()*1000)
                if self.path=='/infer':
                    result=dict(envelope,objects=[]) if args.demo else dict(envelope,**infer(request))
                elif self.path in ('/plan','/chat','/v1/chat/completions'):
                    proposal=dict(explanation='这是协议联调示例：仅生成等待1秒的程序，不识别物体、不规划真实动作。',program=demo_program()) if args.demo else plan(request)
                    result=dict(choices=[dict(message=dict(role='assistant',content=json.dumps(proposal,ensure_ascii=False)))]) if self.path=='/v1/chat/completions' else dict(envelope,**proposal)
                else:self.reply(404,dict(error='unknown route'));return
                self.reply(200,result)
            except NotImplementedError as error:self.reply(501,dict(error=str(error)))
            except (ValueError,KeyError,TypeError) as error:self.reply(400,dict(error=str(error)))
    print(f'Adapter http://127.0.0.1:{args.port} · demo={args.demo}',flush=True)
    ThreadingHTTPServer(('127.0.0.1',args.port),Handler).serve_forever()


if __name__=='__main__':main()
