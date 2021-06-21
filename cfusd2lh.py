import cfusdlog
import argparse
import numpy as np
import yaml
import os.path
from contextlib import redirect_stdout
import numpy as np
from transforms3d import quaternions
from math import tan

def rowify(data):
    keys = data.keys()
    ts = data[list(keys)[0]]
    rtn = []
    for idx, _ in enumerate(ts):
        row = {}
        for key in keys:
            row[key] = data[key][idx]
        rtn.append(row)
    return rtn
    
def generateSweepLines(lhAngleData, gen):
    rows = rowify(lhAngleData)
    lines = []
    for r in rows:
        t = r['timestamp'] / 1000.0
        timecode = round((t * 48000000) % (1 << 32))
        sweep = r['sweep']
        angle = r['angle'] if sweep else r['angle']
        lines.append((t, f"CF0 B {r['basestation']} {r['sensor']} {timecode} {sweep} {angle}"))
    return lines

def generateExtPose(frame):
    rows = rowify(frame)
    lines = []
    for r in rows:
        t = r['timestamp'] / 1000.0
        timecode = round((t * 48000000) % (1 << 32))
        lines.append((t, f"EXT_CF0 EXTERNAL_POSE {r['stateEstimate.x']} {r['stateEstimate.y']} {r['stateEstimate.z']} 0 0 0 0"))
    return lines

def generateIMULines(imuData):
    rows = rowify(imuData)
    lines = []
    for r in rows:
        t = r['timestamp'] / 1000.0
        timecode = round((t * 48000000) % (1 << 32))
        lines.append((t, f"CF0 I 3 {timecode} {r['acc.x']} {r['acc.y']} {r['acc.z']} {r['gyro.x']*0.0174533} {r['gyro.y']*0.0174533} {r['gyro.z']*0.0174533} 0 0 0 0"))
    return lines

def generateUartFrames(uartFrames):
    rows = rowify(uartFrames)
    lines = []
    for r in rows:
        t = r['timestamp'] / 1000.0
        timecode = round((t * 48000000) % (1 << 32))
        lines.append((t, f"CF0 W {r['basestation']} {r['sensor']} {timecode} {r['offset']} {r['timestampFPGA']} {r['timestamp2FPGA']}"))
    return lines
    
"""
fixedFrequency
	timestamp
	lighthouse.delta
	stateEstimate.x
	stateEstimate.y
	stateEstimate.z
	gyro.x
	gyro.y
	gyro.z
	acc.x
	acc.y
	acc.z
activeMarkerModeChanged
	timestamp
	mode
lhAngle
	timestamp
	sensor
	basestation
	sweep
	angle
	correctedAngle
lhUartFrame
	timestamp
	sensor
	basestation
	offset
	timestampFPGA
	timestamp2FPGA

"""

def generate_lh(calibs, geos):
    keys = {
        'phase': 'phase',
#        'tilt': 'tilt',
        'curve': 'curve',
        'gibpha': 'gibphase',
        'gibmag': 'gibmag',
        'ogeephase': 'ogeephase',
        'ogeemag': 'ogeemag'
        }

    for k in keys:
        l = keys[k]        
        print(f'"fcal{k}":["{calibs["sweeps"][0][l]}","{calibs["sweeps"][1][l]}"],')
    l = "tilt"
    print(f'"fcal{k}":["{tan(calibs["sweeps"][0][l])}","{tan(calibs["sweeps"][1][l])}"],')
        
    pos = geos['origin']
    rot = geos['rotation']
    M = np.array(rot)

    q = [.5, .5, -.5, -.5]
    
    quat = quaternions.qmult(quaternions.mat2quat(M), q)

    print(f'"pose":["{pos[0]}","{pos[1]}","{pos[2]}","{quat[0]}","{quat[1]}","{quat[2]}","{quat[3]}"],')
    print('"OOTXSet":"1",')
    print('"PositionSet":"1",')    
    print(f'"id":"{calibs["uid"]}"')

def generate_config(fn):
    name = os.path.basename(args.filename)
    with open(os.path.dirname(fn) + "/system-config.yaml", 'r') as stream:
        try:
            configyml = yaml.safe_load(stream)
            
            with open(name + '.rec.json', 'w') as f:
                with redirect_stdout(f):
                    for idx, calib in enumerate(configyml['calibs']):
                        print(f'"lighthouse{idx}":{{')
                        print(f'"index":"{idx}",')
                        print(f'"mode":"{idx}",')                        
                        generate_lh(configyml['calibs'][calib], configyml['geos'][calib])
                        print(f'}},')
                    print(f'"configed-lighthouse-gen": "{configyml["systemType"]}"')
                return configyml["systemType"]
        except yaml.YAMLError as exc:
            print(exc)
            
            
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filename")
    args = parser.parse_args()
    data = cfusdlog.decode(args.filename)
    
    lhAngleData = data['lhAngle']
    imuData = data['fixedFrequency']

    gen = generate_config(args.filename)
        
    sweepLines = generateSweepLines(lhAngleData, gen)
    imuLines = generateIMULines(imuData)
    uartFrames = generateUartFrames(data['lhUartFrame'])

    name = os.path.basename(args.filename)


    
    file = open('cf0.json',mode='r')
    conf = file.read().replace("\n", "").replace("\r", "")
    
    lines = generateExtPose(imuData) + imuLines + sweepLines
    lines.sort(key=lambda x: x[0])

    with open(name + '.rec', 'w') as f:
        with redirect_stdout(f):
            print(f"0.00000 CF0 CONFIG\t {conf}")
            for l in lines:
                print(f"{l[0]} {l[1]}")
    print(max(lhAngleData['angle']))
    print(min(lhAngleData['angle']))
