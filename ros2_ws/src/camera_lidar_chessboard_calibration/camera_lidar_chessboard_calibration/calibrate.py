import argparse, json
from pathlib import Path
import cv2, numpy as np, yaml

def solve_planes(nc,nl,dc,dl,indices=None):
    idx=np.arange(len(nc)) if indices is None else np.asarray(indices)
    H=nc[idx].T@nl[idx]; U,_,Vt=np.linalg.svd(H)
    R=U@np.diag([1,1,np.linalg.det(U@Vt)])@Vt
    # With x_camera = R @ x_lidar + t and plane equations n.x+d=0:
    # n_camera = R @ n_lidar and d_lidar = d_camera + n_camera.t.
    t=np.linalg.lstsq(nc[idx],dl[idx]-dc[idx],rcond=None)[0]
    return R,t

def residuals(nc,nl,dc,dl,R,t):
    angle=np.degrees(np.arccos(np.clip(np.sum(nc*(nl@R.T),axis=1),-1,1)))
    distance=np.abs(nc@t-(dl-dc))
    return angle,distance

def robust_solve(nc,nl,dc,dl,rounds=3000):
    rng=np.random.default_rng(42); best=np.empty(0,int); best_cost=np.inf
    for _ in range(rounds):
        idx=rng.choice(len(nc),4,replace=False)
        if np.linalg.matrix_rank(nc[idx])<3: continue
        R,t=solve_planes(nc,nl,dc,dl,idx); a,d=residuals(nc,nl,dc,dl,R,t)
        if np.linalg.norm(t)>2.0: continue
        inside=np.flatnonzero((a<6.0)&(d<0.18)); cost=np.sum(a[inside]/6+d[inside]/.18)
        if len(inside)>len(best) or (len(inside)==len(best) and cost<best_cost): best,best_cost=inside,cost
    if len(best)<5: raise RuntimeError(f'Robust solver found only {len(best)} consistent samples; need at least 5')
    for _ in range(3):
        R,t=solve_planes(nc,nl,dc,dl,best); a,d=residuals(nc,nl,dc,dl,R,t)
        refined=np.flatnonzero((a<6.0)&(d<0.18))
        if np.array_equal(refined,best): break
        best=refined
    if np.linalg.norm(t)>2.0: raise RuntimeError('Best solution violates the 2 m camera-LiDAR mounting-distance limit')
    return R,t,best

def camera_plane(meta,K,D):
    cols,rows=meta['board_cols'],meta['board_rows']; s=meta['square_size']
    obj=np.zeros((cols*rows,3),np.float32); obj[:,:2]=np.mgrid[0:cols,0:rows].T.reshape(-1,2)*s
    ok,rvec,t=cv2.solvePnP(obj,np.asarray(meta['image_points'],np.float32),K,D,flags=cv2.SOLVEPNP_IPPE)
    if not ok: raise RuntimeError('solvePnP failed')
    R,_=cv2.Rodrigues(rvec); n=R[:,2]; d=-n.dot(t[:,0])
    if d>0: n,d=-n,-d
    return n,d

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--samples',default='calibration_samples')
    ap.add_argument('--intrinsics',required=True); ap.add_argument('--output',default='lidar_to_camera.yaml')
    ap.add_argument('--no-robust',action='store_true'); a=ap.parse_args()
    fs=cv2.FileStorage(a.intrinsics,cv2.FILE_STORAGE_READ)
    K=fs.getNode('camera_matrix').mat(); D=fs.getNode('distortion_coefficients').mat()
    if K is None: K=fs.getNode('K').mat()
    if D is None: D=fs.getNode('dist_coeffs').mat()
    if K is None: raise SystemExit('Intrinsic YAML has no camera_matrix/K')
    metas=[json.loads(p.read_text(encoding='utf-8')) for p in sorted(Path(a.samples).glob('*.json'))]
    if len(metas)<6: raise SystemExit('Need at least 6 board poses with varied orientations')
    nc=[]; nl=[]; dc=[]; dl=[]
    for m in metas:
        n,d=camera_plane(m,K,D); l=np.asarray(m['lidar_plane_n'],float); ld=float(m['lidar_plane_d'])
        nc.append(n); nl.append(l); dc.append(d); dl.append(ld)
    nc,nl,dc,dl=np.asarray(nc),np.asarray(nl),np.asarray(dc),np.asarray(dl)
    if a.no_robust: R,t,inliers=(*solve_planes(nc,nl,dc,dl),np.arange(len(nc)))
    else: R,t,inliers=robust_solve(nc,nl,dc,dl)
    normal_all,dist_all=residuals(nc,nl,dc,dl,R,t); normal_err=normal_all[inliers]; dist_err=dist_all[inliers]
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=t
    out={'parent_frame':'camera_optical_frame','child_frame':'livox_frame','T_lidar_to_camera':T.tolist(),
         'rotation_matrix':R.tolist(),'translation_m':t.tolist(),'samples':len(metas),
         'inlier_samples':len(inliers),'inlier_files':[Path(sorted(Path(a.samples).glob('*.json'))[i]).name for i in inliers],
         'rejected_samples':len(metas)-len(inliers),
         'normal_rmse_deg':float(np.sqrt(np.mean(normal_err**2))),
         'plane_distance_rmse_m':float(np.sqrt(np.mean(dist_err**2)))}
    Path(a.output).write_text(yaml.safe_dump(out,sort_keys=False),encoding='utf-8')
    print(yaml.safe_dump(out,sort_keys=False))

if __name__=='__main__': main()
