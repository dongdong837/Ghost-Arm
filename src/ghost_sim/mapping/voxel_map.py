"""Conservative known-world voxel map; indices and arrays are ordered [x,y,z]."""
from pathlib import Path
import hashlib
import json
import numpy as np
import xml.etree.ElementTree as ET

from ghost_sim.paths import WORLD, MAPS
FREE, CLEARANCE, OCCUPIED = 0, 1, 2


def build(resolution=0.1, margin=0.05):
    if not np.isfinite(resolution) or resolution<=0 or not np.isfinite(margin) or margin<0:
        raise ValueError('Resolution must be positive and margin nonnegative')
    source = WORLD
    world = ET.parse(source).find('world')
    origin = np.array([-4.,-3.,0.])
    upper = np.array([4.,3.,3.])
    shape = np.rint((upper-origin)/resolution).astype(int)
    if resolution<=0 or not np.allclose(shape*resolution,upper-origin) or margin<0:
        raise ValueError('Resolution must divide room dimensions; margin must be nonnegative')
    radius = float(world.findtext("model[@name='ghost']/link/collision/geometry/sphere/radius"))
    centres = np.stack(np.meshgrid(*[origin[i]+(np.arange(shape[i])+.5)*resolution for i in range(3)],indexing='ij'),axis=-1)
    occupied = np.zeros(tuple(shape),dtype=bool)
    blocked = np.any((centres-resolution/2-origin<radius+margin) | (upper-centres-resolution/2<radius+margin),axis=-1)
    boxes = []
    for model in world.findall('model'):
        if model.findtext('static')!='true':
            continue
        model_pose = np.array(list(map(float,model.findtext('pose','0 0 0 0 0 0').split())))
        for link in model.findall('link'):
            link_pose=np.array(list(map(float,link.findtext('pose','0 0 0 0 0 0').split())))
            for collision in link.findall('collision'):
                local=np.array(list(map(float,collision.findtext('pose','0 0 0 0 0 0').split())))
                if any(not np.allclose(p[3:],0) for p in [model_pose,link_pose,local]):
                    raise ValueError('This map builder requires axis-aligned boxes')
                size=collision.findtext('geometry/box/size')
                if size is None:
                    raise ValueError('Unsupported static collision geometry: '+model.get('name'))
                centre=model_pose[:3]+link_pose[:3]+local[:3]
                half=np.array(list(map(float,size.split())))/2
                boxes.append({'name':model.get('name'),'centre':centre.tolist(),'half_size':half.tolist()})
                # Distance between each whole voxel and obstacle box, not just its centre.
                delta=np.maximum(np.abs(centres-centre)-half-resolution/2,0)
                distance=np.linalg.norm(delta,axis=-1)
                occupied |= distance<=1e-9
                blocked |= distance<=radius+margin+1e-9
    states=np.where(occupied,OCCUPIED,np.where(blocked,CLEARANCE,FREE)).astype(np.uint8)
    metadata={'version':1,'frame':'simulation_reference','resolution':resolution,
              'origin':origin.tolist(),'upper_exclusive':upper.tolist(),'shape_xyz':shape.tolist(),
              'axis_order':'xyz','ghost_radius':radius,'margin':margin,
              'labels':{'0':'free_for_entire_ghost','1':'clearance_exclusion','2':'physical_obstacle'},
              'source':'known SDF collision geometry; not SLAM','sdf_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
              'counts':{str(i):int((states==i).sum()) for i in range(3)},'boxes':boxes}
    return states,centres,metadata


def save():
    states,centres,meta=build()
    np.savez_compressed(MAPS/'flight_space.npz',states=states,metadata=json.dumps(meta))
    (MAPS/'flight_space.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n')
    return states,centres,meta


def index_of(point, metadata):
    # Suppress floating-point artefacts such as 1.2 / 0.1 == 11.999999999999998.
    scaled=(np.asarray(point)-metadata['origin'])/metadata['resolution']
    return np.minimum(np.floor(np.round(scaled,10)).astype(int),np.array(metadata['shape_xyz'])-1)


def query(x,y,z):
    with np.load(MAPS/'flight_space.npz',allow_pickle=False) as data:
        states=data['states']; meta=json.loads(str(data['metadata']))
    if hashlib.sha256((WORLD).read_bytes()).hexdigest()!=meta['sdf_sha256']:
        raise ValueError('Map is stale: regenerate it after changing ghost_room.sdf')
    p=np.array([x,y,z],dtype=float)
    if not np.isfinite(p).all() or np.any(p< meta['origin']) or np.any(p>=meta['upper_exclusive']):
        return {'flyable':False,'state':'outside_map','point':p.tolist()}
    index=index_of(p,meta)
    label=int(states[tuple(index)])
    return {'flyable':label==FREE,'state':meta['labels'][str(label)],'voxel_xyz':index.tolist(),'point':p.tolist()}


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description='Build map, or query a world XYZ point in metres')
    parser.add_argument('--query',nargs=3,type=float)
    args=parser.parse_args()
    if args.query is not None:
        print(json.dumps(query(*args.query),ensure_ascii=False,indent=2))
    else:
        print(json.dumps(save()[2],ensure_ascii=False,indent=2))
