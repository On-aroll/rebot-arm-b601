#!/usr/bin/env python3
"""Inspect existing TCP listeners only; never connect, stop or restart a service."""
import ipaddress
import json
import subprocess
import sys

PORTS = {3002:'web', 3444:'web-https', 9090:'rosbridge', 8081:'mcp', 8082:'text-agent', 9002:'motorbridge'}


def analyze(output):
    listeners = []
    for line in output.splitlines():
        fields = line.split()
        if len(fields) < 5 or fields[0] != 'LISTEN':continue
        host, sep, port = fields[3].rpartition(':')
        if not sep or not port.isdigit() or int(port) not in PORTS:continue
        host = host.strip('[]');port = int(port)
        try:
            address = ipaddress.ip_address(host.split('%')[0])
            local = address.is_loopback or bool(getattr(address, 'ipv4_mapped', None) and address.ipv4_mapped.is_loopback)
        except ValueError:local = False
        listeners.append(dict(service=PORTS[port], address=host, port=port, loopback_only=local))
    exposed = [r for r in listeners if not r['loopback_only']]
    return dict(schema_version='b601-network-audit/1.0', listeners=listeners, exposed=exposed,
        web_present=any(r['port'] in (3002, 3444) for r in listeners),
        rosbridge_present=any(r['port'] == 9090 for r in listeners),
        observed_listeners_loopback_only=bool(listeners) and not exposed,
        actions_performed=[], authentication_verified=False, dds_security_verified=False,
        message='non-loopback listener found; controlled reload/firewall review required' if exposed else
                'observed listeners are local only; this does not verify authentication' if listeners else 'no selected listeners found')


if __name__ == '__main__':
    try:
        output = subprocess.check_output(['ss', '-H', '-ltn'], text=True, stderr=subprocess.PIPE, timeout=5)
        report = analyze(output)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        sys.exit(1 if report['exposed'] else 0 if report['web_present'] and report['rosbridge_present'] else 2)
    except (OSError, subprocess.SubprocessError) as error:
        print(f'network audit unavailable: {error}', file=sys.stderr);sys.exit(3)
