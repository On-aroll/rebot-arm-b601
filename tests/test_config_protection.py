"""Offline contract/fault tests; temporary folders only; never imports a driver."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from rebot_config import ConfigStore, ConfigAPI, blank_fields, envelope, DEVICE_UNITS, preparation_checks


class Protection(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.store = ConfigStore(self.tmp.name)

    def act(self, action, **kw):
        return self.store.action(dict(action=action, revision=self.store.load()['revision'], **kw))

    def draft(self, value=2):
        fields = blank_fields(); fields['kp']['value'] = value
        return self.act('draft', axis_id=1, fields=fields)

    def approve(self):
        token = self.act('review', axis_id=1)['review_token']
        return self.act('approve', review_token=token, confirm='software-only')

    def test_restart_separation_and_nulls(self):
        state = self.store.status()
        self.assertFalse(state['hardware']); self.assertFalse(state['armed']); self.assertTrue(state['drill'])
        self.assertEqual(state['state']['approved'], {})
        self.draft(); self.approve(); self.draft(3)
        store = ConfigStore(self.tmp.name); state = store.status()
        self.assertEqual(state['state']['approved']['1']['fields']['kp']['value'], 2)
        self.assertEqual(state['state']['drafts']['1']['fields']['kp']['value'], 3)
        self.assertIsNone(state['state']['approved']['1']['fields']['a_max']['value'])
        self.assertEqual(state['execution_state'], 'LOCKED')
        self.assertEqual((Path(self.tmp.name)/'machine_profile.json').stat().st_mode & 0o777, 0o600)

    def test_invalid_fields_units_nan_types(self):
        for value in (float('nan'),float('inf'),True,'12',-1):
            with self.assertRaises(ValueError): self.draft(value)
        for fields in ({}, {**blank_fields(),'kp':{'unit':'A','value':1}}):
            with self.assertRaises(ValueError): self.act('draft',axis_id=1,fields=fields)
        with self.assertRaises(ValueError): self.act('draft',axis_id=True,fields=blank_fields())
        self.assertEqual(self.store.load()['revision'],0)

    def test_review_conflict_replay_expiry_and_session(self):
        self.draft(); token = self.act('review',axis_id=1)['review_token']; self.draft(3)
        with self.assertRaises(ValueError): self.act('approve',review_token=token,confirm='software-only')
        with self.assertRaises(ValueError): self.store.action(dict(action='draft',revision=0,axis_id=1,fields=blank_fields()))
        token=self.act('review',axis_id=1)['review_token']
        self.store.reviews[token]['expires'] = 0
        with self.assertRaises(ValueError): self.act('approve',review_token=token,confirm='software-only')
        token=self.act('review',axis_id=1)['review_token']
        with self.assertRaises(ValueError): self.store.action(dict(action='approve',revision=self.store.load()['revision'],review_token=token,confirm='software-only'),session='other-tab')
        token=self.act('review',axis_id=1)['review_token']
        self.act('approve',review_token=token,confirm='software-only')
        with self.assertRaises(ValueError): self.act('approve',review_token=token,confirm='software-only')

    def test_atomic_failure_preserves_approved(self):
        self.draft(); self.approve(); before=self.store.load()
        with patch('rebot_config.os.replace',side_effect=OSError('disk failure')):
            with self.assertRaises(OSError): self.draft(9)
        self.assertEqual(self.store.load(),before)
        with patch('rebot_config.os.open',side_effect=PermissionError('denied')):
            self.assertFalse(self.store.status()['hardware'])
        self.assertEqual(self.store.load(),before)

    def test_corruption_missing_head_incompatible(self):
        self.draft(); self.approve()
        path=Path(self.tmp.name)/'machine_profile.json'; old=path.read_text()
        for value in ('{broken', json.dumps(envelope({**self.store.load(),'schema_version':999}))):
            path.write_text(value)
            self.assertIsNone(self.store.status()['state'])
            self.assertFalse(self.store.status()['hardware'])
            with self.assertRaises(ValueError): self.store.action(dict(action='draft',revision=0,axis_id=1,fields=blank_fields()))
            path.write_text(old)
        path.unlink()
        self.assertIn('丢失',self.store.status()['error'])

    def test_import_and_mock_do_not_promote_or_replace_approval(self):
        self.draft(); self.approve(); backup=self.act('export')['backup']
        self.draft(4); self.approve(); approved=copy.deepcopy(self.store.load()['approved'])
        self.act('import',backup=backup)
        self.assertEqual(self.store.load()['approved'],approved)
        self.assertEqual(self.store.load()['drafts']['1']['fields']['kp']['value'],2)
        for outcome in ('match','mismatch','timeout'):
            self.act('mock_apply',axis_id=1,outcome=outcome)
            self.assertEqual(self.store.load()['approved'],approved)
        bad=copy.deepcopy(backup); bad['state']['device_id']='another-device'
        with self.assertRaises(ValueError): self.act('import',backup=envelope(bad['state']))
        bad['checksum']='wrong'
        with self.assertRaises(ValueError): self.act('import',backup=bad)

    def test_read_failure_snapshot_keeps_unknown_and_never_unlocks(self):
        snapshot=dict(schema_version=1,axis_id=1,model='rs-06',source='mock',measured_at=1,control_mode=0,review_status='collected',identity_status='unverified',device_id=None,
          fields={k:dict(value=None,unit=u,measured_at=None,status='error') for k,u in DEVICE_UNITS.items()})
        self.act('snapshot',snapshot=snapshot)
        self.assertEqual(self.store.load()['approved'],{})
        checks=preparation_checks(self.store.load(),now=100)[0]
        self.assertFalse(checks['ready'])
        self.assertTrue(any('过期' in reason for reason in checks['reasons']))
        self.assertTrue(any('Mock' in reason for reason in checks['reasons']))
        self.assertFalse(self.store.status()['hardware'])
        snapshot['fields']['limitSpd']['value']=0.1
        with self.assertRaises(ValueError): self.act('snapshot',snapshot=snapshot)
        for op in ('enable','disable','stop','ensure_mode','mit','pos_vel','vel','robstride_write_param','set_zero'):
            with self.assertRaises(ValueError): self.act(op)

    def test_explicit_software_recovery_preserves_corrupt_head(self):
        from rebot_config_restore import restore
        self.draft(); self.approve()
        expected=self.store.load()
        record=next((Path(self.tmp.name)/'history').glob('00000002-*.json'))
        (Path(self.tmp.name)/'machine_profile.json').write_text('{broken')
        self.assertEqual(restore(self.tmp.name,record),2)
        self.assertEqual(self.store.load(),expected)
        self.assertEqual(len(list(Path(self.tmp.name).glob('pre-recovery-*.json'))),1)
        self.assertFalse(self.store.status()['hardware'])

    def test_local_api_auth_origin_replay_and_direct_hardware(self):
        api=ConfigAPI(self.store,8765)
        headers={'Host':'127.0.0.1:8765','Origin':'http://127.0.0.1:8765'}
        def request(body, custom=None, peer='127.0.0.1',path='/config/action'):
            return api.request('POST',path,custom or headers,peer,body)
        for h in ({**headers,'Origin':'https://evil.test'}, {'Host':headers['Host']}, {**headers,'Host':'public.lhr.life'}, {**headers,'Sec-Fetch-Site':'cross-site'}):
            with self.assertRaises(ValueError): request({},h,path='/config/session')
        with self.assertRaises(ValueError): request({},peer='192.168.1.5',path='/config/session')
        with self.assertRaises(ValueError): request({'action':'status','seq':1})
        result=request({},path='/config/session'); headers['X-Config-Session']=result['token']
        self.assertFalse(request({'action':'status','seq':1})['hardware'])
        with self.assertRaises(ValueError): request({'action':'status','seq':1})
        for seq,op in enumerate(('enable','mit','stop','set_zero'),2):
            with self.assertRaises(ValueError): request({'action':op,'seq':seq,'revision':0,'armed':True,'hardware':True})
        api.sessions[result['token']]['expires']=0
        with self.assertRaises(ValueError): request({'action':'status','seq':6})

if __name__=='__main__': unittest.main(verbosity=2)
