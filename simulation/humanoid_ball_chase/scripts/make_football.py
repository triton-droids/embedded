"""Generate black pentagon / white hexagon truncated-icosahedron visual meshes."""
from itertools import combinations
from pathlib import Path
import math
import struct
import numpy as np


def generate(output):
    phi=(1+math.sqrt(5))/2
    vertices=np.array([(0,a,b*phi) for a in (-1,1) for b in (-1,1)]+
                      [(a,b*phi,0) for a in (-1,1) for b in (-1,1)]+
                      [(b*phi,0,a) for a in (-1,1) for b in (-1,1)],float)
    edges={(i,j) for i in range(12) for j in range(12)
           if i!=j and np.isclose(np.linalg.norm(vertices[i]-vertices[j]),2)}
    truncated={(i,j):(2*vertices[i]+vertices[j])/3 for i,j in edges}
    scale=.1/np.linalg.norm(next(iter(truncated.values())))
    polygons={'black':[], 'white':[]}
    for i in range(12):
        polygons['black'].append([truncated[(i,j)] for j in range(12) if (i,j) in edges])
    for i,j,k in combinations(range(12),3):
        if all(pair in edges for pair in ((i,j),(j,k),(k,i))):
            polygons['white'].append([truncated[p] for p in ((i,j),(j,i),(j,k),(k,j),(k,i),(i,k))])
    for color,faces in polygons.items():
        triangles=[]
        for face in faces:
            face=np.array(face)*scale
            center=face.mean(axis=0);normal=center/np.linalg.norm(center)
            x=face[0]-center;x/=np.linalg.norm(x);y=np.cross(normal,x)
            angles=np.arctan2((face-center)@y,(face-center)@x)
            face=face[np.argsort(angles)]
            for i in range(len(face)):
                triangles.append([center,face[i],face[(i+1)%len(face)]])
        with (output/f'football_{color}.stl').open('wb') as f:
            f.write(b'Procedural truncated icosahedron football; metres'.ljust(80,b'\0'))
            f.write(struct.pack('<I',len(triangles)))
            for triangle in triangles:f.write(struct.pack('<12fH',0,0,0,*np.array(triangle).ravel(),0))
    assert len(polygons['black'])==12 and len(polygons['white'])==20


if __name__=='__main__':generate(Path(__file__).resolve().parents[1]/'assets')
