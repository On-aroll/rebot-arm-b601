"""Run the robot's one continuous 3-D 国庆快乐 path on the local simulator."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from rebot_client import RebotClient

if __name__=='__main__':
    client=RebotClient()
    client.control('reset')
    client.control('camera_writing')
    result=client.control('print_demo')
    print('连续书写任务：',result['job']['id'],flush=True)
    print(client.wait(result['job']['id'],timeout=300)['status'])
