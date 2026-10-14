"""Fixed loopback bridge to the isolated upstream ROS Fake validation service."""
import json
import urllib.request
import urllib.error

def ros_request(action=None, **body):
    url='http://127.0.0.1:8767/'+('action' if action else 'state')
    data=json.dumps(dict(action=action,**body)).encode() if action else None
    try:
        with urllib.request.urlopen(urllib.request.Request(url,data,{'Content-Type':'application/json'}),timeout=4) as response:
            result=json.load(response)
    except urllib.error.HTTPError as error:
        raise ValueError(json.load(error).get('error','ROS 验证请求失败')) from error
    except OSError as error:
        raise ValueError('ROS 虚拟验证服务未启动：在项目目录运行 bash scripts/start_ros_validation.sh') from error
    if result.get('backend')!='ros2-fake' or result.get('namespace')!='rebotarm_validation' or result.get('hardware') is not False:
        raise ValueError('不是已隔离的 ROS 虚拟设备，拒绝连接')
    return result
