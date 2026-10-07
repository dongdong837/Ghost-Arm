"""Real simulation acceptance: vertical flight, travel over a box and rejection."""
import json,subprocess
from pathlib import Path
import numpy as np
from ghost_sim.navigation.planner3d import Planner
from ghost_sim.paths import SCRIPTS, REPORTS
planner=Planner();reports=[]
for label,goal in [('vertical',[0,0,2.2]),('approach',[.8,1,.5]),('over_box',[2.8,1,.5])]:
    result=subprocess.run([str(SCRIPTS/'navigate.sh'),*map(str,goal)],timeout=195)
    assert result.returncode==0,label
    data=json.loads((REPORTS/'navigation3d_result.json').read_text())
    assert data['status']=='SUCCEEDED' and data['position_error_m']<.08,data
    points=[p[:3] for p in data['trajectory']]
    assert len(points)>5
    assert all(planner.segment_clear(a,b) for a,b in zip(points,points[1:])), 'Recorded trajectory crosses clearance boundary'
    if label=='over_box':assert max(p[2] for p in points)>.95,'Did not fly over the box'
    (REPORTS/f'navigation3d_{label}.json').write_text(json.dumps(data,indent=2)+'\n')
    reports.append({'case':label,'error_m':data['position_error_m'],'min_z':min(p[2] for p in points),'max_z':max(p[2] for p in points),'samples':len(points)})
result=subprocess.run([str(SCRIPTS/'navigate.sh'),'1.8','1','.3'],timeout=20)
assert result.returncode!=0,'Obstacle goal unexpectedly accepted'
report={'result':'PASS','goals':reports,'obstacle_goal_rejected':True}
(REPORTS/'navigation3d_validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
