import json
import shutil
import subprocess
from pathlib import Path
import struct
import tempfile
import time
import unittest
import zlib
from unittest.mock import patch
from urllib.request import Request, urlopen
from apps.monitor.joint_monitor import JointMonitor

from apps.monitor.depth_recording import DepthRecorder, FRAME, MAGIC, encode_depth, read_recording


class Preview:
    def __init__(self):
        self.sequence=0
        self.fresh=True
    def process(self, query):
        self.sequence+=1
        return dict(error=None,fresh=self.fresh,image=[.123456789,-.0,1.0,2.5],width=2,height=2,
                    units='m',options={'normalize':False},session='123',sequence=self.sequence,
                    captured_at=self.sequence*.02,age_ms=0,source='simulation')


class DepthRecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.preview=Preview()
        self.recorder=DepthRecorder(self.preview,self.temp.name)
    def tearDown(self):
        self.recorder.stop();self.temp.cleanup()
    def record(self):
        self.recorder.start({},10)
        time.sleep(.25)
        self.recorder.stop()
        state=self.recorder.status()
        self.assertIsNone(state['error'])
        self.assertGreaterEqual(state['frames'],2)
        return self.recorder.download_path(state['filename'])
    def test_lossless_roundtrip_metadata_and_restart(self):
        path=self.record()
        records=list(read_recording(path))
        self.assertEqual(records[0]['units'],'m')
        self.assertEqual(records[0]['options'],{'normalize':False})
        expected=struct.pack('<4f',.123456789,-.0,1.0,2.5)
        for frame in records[1:]:
            self.assertEqual(frame['data'],expected)
            self.assertEqual(frame['source'],'simulation')
            self.assertEqual(frame['session'],'123')
        self.assertEqual(len(records)-1,self.recorder.status()['frames'])
        self.assertEqual(DepthRecorder(self.preview,self.temp.name).status()['files'][0]['name'],path.name)
    def test_partial_tail_recovery_and_checksum(self):
        path=self.record();original=path.read_bytes()
        path.write_bytes(original[:-1])
        with self.assertRaises(ValueError):list(read_recording(path))
        self.assertGreaterEqual(len(list(read_recording(path,recover=True))),2)
        path.write_bytes(original)
        offset=12+struct.unpack('<I',original[8:12])[0]
        broken=bytearray(original);broken[offset+FRAME.size-1]^=1;path.write_bytes(broken)
        with self.assertRaisesRegex(ValueError,'checksum'):list(read_recording(path))
    def test_validation_and_no_active_file_download(self):
        for fps in (0,4,31,float('nan'),True,'5'):
            with self.assertRaises(ValueError):self.recorder.start({},fps)
        self.preview.fresh=False
        with self.assertRaises(ValueError):self.recorder.start({})
        self.preview.fresh=True;self.recorder.start({})
        with self.assertRaises(ValueError):self.recorder.start({})
        with self.assertRaises(ValueError):self.recorder.download_path(self.recorder.status()['filename'])
        with self.assertRaises(FileNotFoundError):self.recorder.download_path('../outside')
    def test_disk_failure_is_reported(self):
        with patch('apps.monitor.depth_recording.encode_depth',side_effect=OSError('disk failure')):
            self.recorder.start({})
            self.recorder.stop()
            # If stop preceded the first frame, no write was attempted; run again and wait.
            self.recorder.start({});time.sleep(.1);self.recorder.stop()
        self.assertIn('disk failure',self.recorder.status()['error'])
    def test_http_record_download_and_read(self):
        monitor=JointMonitor('127.0.0.1',0,record_dir=self.temp.name,preview_port=0)
        monitor.depth_recorder.preview=self.preview
        monitor.start()
        base='http://127.0.0.1:'+str(monitor.server.server_port)
        def post(action,body):
            with urlopen(Request(base+'/api/depth-recordings/'+action,
                         data=json.dumps(body).encode(),headers={'Content-Type':'application/json'}),timeout=3) as response:
                return json.load(response)
        try:
            started=post('start',{'options':{},'fps':10})
            self.assertTrue(started['recording'])
            time.sleep(.25)
            stopped=post('stop',{})
            self.assertGreaterEqual(stopped['frames'],2)
            with urlopen(base+'/api/depth-recordings/download?name='+stopped['filename']) as response:
                data=response.read()
                self.assertEqual(response.headers['Content-Type'],'application/octet-stream')
            downloaded=Path(self.temp.name)/'download.g1depth';downloaded.write_bytes(data)
            self.assertEqual(len(list(read_recording(downloaded)))-1,stopped['frames'])
        finally:monitor.close()

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'), 'FFmpeg needed')
    def test_30fps_video_and_lossless_data_have_same_frame_count(self):
        class VideoPreview(Preview):
            def process(inner,query):
                frame=super().process(query)
                frame.update(width=3,height=3,image=[.1,.2,.3,.4,.5,.6,.7,.8,.9])
                frame['options'].update(minimum=0,maximum=2.5)
                return frame
        self.recorder.preview=VideoPreview()
        self.recorder.start({},30,video=True,color='color')
        time.sleep(.8)
        self.recorder.stop()
        state=self.recorder.status()
        self.assertIsNone(state['error'],state)
        self.assertGreaterEqual(state['frames'],20)
        self.assertGreater(state['sampled_fps'],25)
        video=self.recorder.download_path(state['filename'].replace('.g1depth','.mp4'))
        result=subprocess.run(['ffprobe','-v','error','-count_frames','-select_streams','v:0',
            '-show_entries','stream=codec_name,width,height,r_frame_rate,nb_read_frames','-of','json',str(video)],capture_output=True,text=True,check=True)
        stream=json.loads(result.stdout)['streams'][0]
        self.assertEqual(stream['codec_name'],'h264')
        self.assertEqual((stream['width'],stream['height']),(4,4))
        self.assertEqual(stream['r_frame_rate'],'30/1')
        self.assertEqual(int(stream['nb_read_frames']),state['frames'])
        frames=list(read_recording(self.recorder.download_path(state['filename'])))
        self.assertEqual(len(frames)-1,state['frames'])
        self.assertEqual(frames[1]['data'],struct.pack('<9f',.1,.2,.3,.4,.5,.6,.7,.8,.9))
        self.assertTrue(state['files'][0]['video'])

    def test_missing_ffmpeg_does_not_prevent_numeric_recording(self):
        with patch('apps.monitor.depth_recording.shutil.which',return_value=None):
            with self.assertRaisesRegex(ValueError,'ffmpeg'):
                self.recorder.start({},5,video=True)
            self.recorder.start({},5,video=False)
            self.recorder.stop()
        self.assertIsNone(self.recorder.status()['error'])

    def test_delta_compresses_repeated_values(self):
        raw=struct.pack('<576f',*([.5]*576))
        self.assertLess(len(encode_depth(raw,raw)),len(raw)//20)


if __name__=='__main__':unittest.main()
